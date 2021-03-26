#ifndef COMMON_H
#define COMMON_H

#include "i2c.h"

#define DELAY_VALUE 900
#define PRESCALER_COUNT 0x1F
#define SCLK_COUNT 0x91

#define I2C i2c_instance[1]
#define DS3231_SLAVE_ADDRESS 0XD0
#define DS3231_REG_OFFSET 0
#define DS3231_DEC_TO_HEX(decimal)  ( ( (decimal / 10 ) << 4) | (decimal % 10) )

#define BMP280_SLAVE_ADDRESS 0xEC  //Defines the Starting address of slave//
#define BMP280_CTRL_MEANS 0xF4
#define BMP280_NORMAL_MODE 0x26
#define BMP280_STATUS_REGISTER 0xF3
#define BMP280_CONFIG_REGISTER 0xF5
#define BMP280_RESET_REGISTER 0xE0

#endif
