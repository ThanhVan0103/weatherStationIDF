/* bh1750.c */
#include "bh1750.h"
#include "driver/i2c.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

static int i2c_write_data(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, I2C_MASTER_NACK);
    if (regAddr != NULL)
    {
        i2c_master_write_byte(cmd, regAddr, I2C_MASTER_NACK);
    }
    i2c_master_write(cmd, pData, dataLen, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int i2c_read_data(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | READ_BIT, I2C_MASTER_NACK);
    if (regAddr != NULL)
    {
        i2c_master_write_byte(cmd, regAddr, I2C_MASTER_NACK);
    }
    i2c_master_read(cmd, pData, dataLen, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bh1750_init(void)
{
    uint8_t data = BH1750_PWR_ON;
    i2c_write_data(BH1750_SLAVE_ADDR, NULL, &data, 1);
    data = BH1750_CON_H;
    i2c_write_data(BH1750_SLAVE_ADDR, NULL, &data, 1); 
}

float bh1750_read_light_intensity(void)
{
    float lux;
    uint8_t sensorData[2] = {0};
    i2c_read_data(BH1750_SLAVE_ADDR, NULL, sensorData, 2);
    lux = (sensorData[0] << 8 | sensorData[1]) / 1.2;
    return lux;
}
