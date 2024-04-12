#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "bh1750.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define MQTT_TOPIC_TEMPERATURE "esp32/bme280/temperature"
#define MQTT_TOPIC_HUMIDITY "esp32/bme280/humidity"
#define MQTT_TOPIC_PRESSURE "esp32/bme280/pressure"
#define MQTT_TOPIC_LIGHT_INTENSITY "esp32/bh1750/light_intensity"
#define MQTT_TOPIC_UV_INDEX "esp32/ml8511/uv_index"
#define MQTT_TOPIC_WIND_SPEED "esp32/a3144/wind_speed"

#define EXAMPLE_ESP_WIFI_SSID "010203"
#define EXAMPLE_ESP_WIFI_PASS "1234567890"
#define MAX_RETRY 10

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static const char *TAG = "MQTT_EXAMPLE";
static const char *TAG_BME280 = "BME280";

static int retry_cnt = 0;

int hallState = 0;
int windMap[2][32] = {
  {2000, 1400, 1000, 600, 500, 400, 320, 280, 240, 220, 200, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 38, 36, 34, 32, 30, 28},
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 23, 25, 29, 34, 41, 51, 54, 57, 60, 64, 68, 73}
};

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_4;     //GPIO32 if ADC1, GPIO13 if ADC2
static const adc_channel_t channel2 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;


void i2c_master_init()
{
    i2c_config_t i2c_config_0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000};
        
    i2c_param_config(I2C_NUM_0, &i2c_config_0);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

uint32_t MQTT_CONNEECTED = 0;

static void mqtt_app_start(void);

static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip: startibg MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
            ESP_LOGI(TAG, "Max Retry Failed: Wi-Fi Connection\n");
        break;

    default:
        break;
    }
    return ESP_OK;
}

void wifi_init(void)
{
    esp_event_loop_create_default();
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        MQTT_CONNEECTED = 1;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNEECTED = 0;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

esp_mqtt_client_handle_t client = NULL;
static void mqtt_app_start(void)
{
    ESP_LOGI(TAG, "STARTING MQTT");
    esp_mqtt_client_config_t mqttConfig = {
        .uri = "mqtt://172.20.112.71:1883"};

    client = esp_mqtt_client_init(&mqttConfig);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void Publisher_Task(void *params)
{
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS1,
        .delay_msec = BME280_delay_msek};

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    if (com_rslt == SUCCESS)
    {
        while (true)
        {
            vTaskDelay(5000 / portTICK_PERIOD_MS);

            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);


            double temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
            char temperature[12];
            sprintf(temperature, "%.2f degC", temp);

            double press = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
            char pressure[10];
            sprintf(pressure, "%.2f hPa", press);

            double hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
            char humidity[10];
            sprintf(humidity, "%.2f %%", hum);

            if (com_rslt == SUCCESS)
            {
                if (MQTT_CONNEECTED)
                {
                    esp_mqtt_client_publish(client, MQTT_TOPIC_TEMPERATURE, temperature, 0, 0, 0);
                    esp_mqtt_client_publish(client, MQTT_TOPIC_PRESSURE, pressure, 0, 0, 0);
                    esp_mqtt_client_publish(client, MQTT_TOPIC_HUMIDITY, humidity, 0, 0, 0);
                }
            }
            else
            {
                ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
                
            }
        }
    }
    else
    {
        ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
    }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADC_Task(void *arg)
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    char uv_Index[20];
    while (1) {
        uint32_t adc_reading_ML8511 = 0;
        uint32_t adc_reading_A3144 = 0;

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading_ML8511 += adc1_get_raw((adc1_channel_t)channel);
            adc_reading_A3144 += adc1_get_raw((adc1_channel_t)channel2);
        }
        adc_reading_ML8511 /= NO_OF_SAMPLES;
        adc_reading_A3144 /= NO_OF_SAMPLES;

        uint32_t voltage_ML8511 = esp_adc_cal_raw_to_voltage(adc_reading_ML8511, adc_chars);
        uint32_t voltage_A3144 = esp_adc_cal_raw_to_voltage(adc_reading_A3144, adc_chars);
      
        // Map voltage to UV index
        //printf("voltage_ML8511: %d mV\n", voltage_ML8511); // In ra giá trị điện áp
        //printf("voltage_A3144: %d mV\n", voltage_A3144);

        int HALL_THRESHOLD = 75;
        hallState = (voltage_A3144 > HALL_THRESHOLD) ? 0 : 1;
        printf("Hall Sensor State: %d\n", hallState);

        float uvIndex = mapfloat(voltage_ML8511/1000, 0.99, 2.8, 0.0, 15.0);
        //printf("UV Intensity (mW/cm^2): %.4f\n", uvIndex); // In ra giá trị điện áp
        snprintf(uv_Index, sizeof(uv_Index), "%.2f ", uvIndex);   // Convert float to string
         if (MQTT_CONNEECTED)
        {
            esp_mqtt_client_publish(client, MQTT_TOPIC_UV_INDEX, uv_Index, 0, 0, 0);

        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void wind_speed_task(void *pvParameters) {
  int counter = 0;
  int lastState = 0;
  float wind = 0;
  char windSpeed_ms[20];

  while (1) {
    // Đếm số lần quay của cảm biến gió trong khoảng thời gian 6 giây
    for (int i = 0; i <= 6000; i++) {
      if (hallState == 1 && lastState == 0) {
        counter++;
        printf("--------------counter: %d\n", counter);
        lastState = 1;
      } else if (hallState == 0 && lastState == 1) {
        lastState = 0;
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
      //vTaskDelay(pdMS_TO_TICKS(1));
    }

    int ave = 2000;
    if (counter > 1) {
      ave = 6000 / (counter - 1);
    }

    // Tìm tốc độ gió tương ứng với thời gian trung bình giữa các vòng quay
    int index = 0;
    while (ave < windMap[0][index]) {
      index++;
    }
    wind = windMap[1][index]; 
    wind = wind * 0.27; // convert kph to ms
    //printf("AVE: %d\n", ave);
    printf("Wind Speed ms: %.2f\n", wind);
    snprintf(windSpeed_ms, sizeof(windSpeed_ms), "%.2f ms", wind);
    if (MQTT_CONNEECTED)
        {
            esp_mqtt_client_publish(client, MQTT_TOPIC_WIND_SPEED, windSpeed_ms, 0, 0, 0);
        }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Chờ 1 giây trước khi tính toán lại tốc độ gió
  }
}

void bh1750_task(void *pvParameters)
{
    bh1750_init();
    char light_intensity[20]; // Buffer to hold the light intensity as a string
    while (1)
    {
        float lux = bh1750_read_light_intensity(); //ESP_LOGI(TAG, "Light intensity: %.2f lux", lux);
        snprintf(light_intensity, sizeof(light_intensity), "%.2f lux", lux);   // Convert float to string
        if (MQTT_CONNEECTED)
        {
            esp_mqtt_client_publish(client, MQTT_TOPIC_LIGHT_INTENSITY, light_intensity, 0, 0, 0);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    i2c_master_init();     

    xTaskCreate(&bh1750_task, "bh1750_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(Publisher_Task, "Publisher_Task", 1024 * 5, NULL, 5, NULL);
    xTaskCreatePinnedToCore(ADC_Task, "ADC_Task", 4096, NULL, 3, NULL, 1);
    xTaskCreate(wind_speed_task, "wind_speed_task", 4096, NULL, 1, NULL);
}
