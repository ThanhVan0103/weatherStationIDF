/* bh1750.h */
#ifndef BH1750_H
#define BH1750_H

#include <stdint.h>


#define BH1750_SLAVE_ADDR   0x23 
#define BH1750_PWR_DOWN     0x00 
#define BH1750_PWR_ON       0x01 
#define BH1750_RST          0x07 
#define BH1750_CON_H        0x10 
#define BH1750_CON_H2       0x11 
#define BH1750_CON_L        0x13
#define BH1750_ONE_H        0x20 
#define BH1750_ONE_H2       0x21
#define BH1750_ONE_L        0x23 

#define WRITE_BIT I2C_MASTER_WRITE             
#define READ_BIT I2C_MASTER_READ

void bh1750_init(void);
float bh1750_read_light_intensity(void);

#endif /* BH1750_H */
