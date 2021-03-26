#include <stdint.h>//Includes the definitions of standard input/output functions//
#include "i2c.h"
#include "log.h"
#include "uart.h"

#include "common.h"
#include "BME280.h"

#define BMP280_REG_DIG_T1 0x88
#define BMP280_REG_DIG_T2 0x8A
#define BMP280_REG_DIG_T3 0x8C

#define BMP280_REG_DIG_P1 0x8E
#define BMP280_REG_DIG_P2 0x90
#define BMP280_REG_DIG_P3 0x92
#define BMP280_REG_DIG_P4 0x94
#define BMP280_REG_DIG_P5 0x96
#define BMP280_REG_DIG_P6 0x98
#define BMP280_REG_DIG_P7 0x9A
#define BMP280_REG_DIG_P8 0x9C
#define BMP280_REG_DIG_P9 0x9E

uint16_t bmp280_calib_dig_T1;
int16_t  bmp280_calib_dig_T2;
int16_t  bmp280_calib_dig_T3;

uint16_t bmp280_calib_dig_P1;
int16_t  bmp280_calib_dig_P2;
int16_t  bmp280_calib_dig_P3;
int16_t  bmp280_calib_dig_P4;
int16_t  bmp280_calib_dig_P5;
int16_t  bmp280_calib_dig_P6;
int16_t  bmp280_calib_dig_P7;
int16_t  bmp280_calib_dig_P8;
int16_t  bmp280_calib_dig_P9;

int read_bmp280_register(i2c_struct *i2c_instance, unsigned int reg_offset, unsigned int *readTemp, unsigned long delay) {
  unsigned char read_buf[4] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  unsigned char temp = 0;
//Writes the slave address for write
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_WRITE, 800);

//Writes the pointer to address that needs to be read
  i2c_write_data(i2c_instance, reg_offset , 1000);

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;

//Writes the slave address for read
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_READ, 800);

/* Make a dummy read as per spec of the I2C controller */
  i2c_read_data(i2c_instance, &temp, 1000);
  i2c_instance->control = I2C_NACK;

//Reads the MSB Byte of temperature [D9 - D1]
  i2c_read_data(i2c_instance, &read_buf[0], 1000);

  i2c_instance->control = I2C_STOP;
  *readTemp = read_buf[0] ;
  return 0;
}

int read_bmp280_values(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned long *pressure, unsigned long *temperature, unsigned long delay) {
  unsigned char read_buf[6] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  int32_t adc_P, adc_T, var1, var2, var3, var4, t_fine, temp;
  int32_t p;

//Writes the slave address for write
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_WRITE, 800);

//Writes the pointer to address that needs to be read
  i2c_write_data(i2c_instance, reg_offset , 1000);

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;

//Writes the slave address for read
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_READ, 800);

/* Make a dummy read as per spec of the I2C controller */
  i2c_read_data(i2c_instance, &temp, 1000);

//Read Pressure
  i2c_read_data(i2c_instance, &read_buf[0], 1000);
  i2c_read_data(i2c_instance, &read_buf[1], 1000);
  i2c_read_data(i2c_instance, &read_buf[2], 1000);
  
//Read Temperature  
  i2c_read_data(i2c_instance, &read_buf[3], 1000);
  i2c_read_data(i2c_instance, &read_buf[4], 1000);
  i2c_instance->control = I2C_NACK;
  i2c_read_data(i2c_instance, &read_buf[5], 1000);
  
  i2c_instance->control = I2C_STOP;
  adc_P = ((read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4));
  adc_T = ((read_buf[3] << 12) | (read_buf[4] << 4) | (read_buf[5] >> 4));

// Calculate TEMPERATURE
  var1 = ((((adc_T / 8) - ((int32_t)bmp280_calib_dig_T1 * 2))) * ((int32_t)bmp280_calib_dig_T2)) / 2048;
    var2 = (((((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1)) * ((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1))) / 4096) * ((int32_t)bmp280_calib_dig_T3)) / 16384;
    t_fine = var1 + var2;
  temp = (t_fine * 5 + 128) / 256;
  *temperature = temp;
  printf("\nTemperature Value:%u.%u °C", (temp/100),(temp%100));

//Calculate Pressure
  var1 = 0; var2 = 0;
  var1 = (((int32_t)t_fine) / 2) - (int32_t)64000;
  var2 = (((var1/4) * (var1/4)) / 2048 ) * ((int32_t)bmp280_calib_dig_P6);
  var2 = var2 + ((var1 * ((int32_t)bmp280_calib_dig_P5)) * 2);
  var2 = (var2/4) + (((int32_t)bmp280_calib_dig_P4) * 65536);
  var1 = ((((int32_t)bmp280_calib_dig_P3 * (((var1/4) * (var1/4)) / 8192 )) / 8) + ((((int32_t)bmp280_calib_dig_P2) * var1)/2)) / 262144;
  var1 =((((32768 + var1)) * ((int32_t)bmp280_calib_dig_P1)) / 32768);

  if (var1 == 0)
    return 0; // avoid exception caused by division by zero

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 / 4096))) * 3125;

  if (p < 0x80000000)
    p = (p * 2) / ((uint32_t)var1);

  else
    p = (p / (uint32_t)var1) * 2;

  var1 = (((int32_t)bmp280_calib_dig_P9) * ((int32_t)(((p/8) * (p/8)) / 8192))) / 4096;
  var2 = (((int32_t)(p/4)) * ((int32_t)bmp280_calib_dig_P8)) / 8192;

  p = (uint32_t)((int32_t)p + ((var1 + var2 + (int32_t)bmp280_calib_dig_P7) / 16));
  *pressure = p;
  printf("\nThe Pressure Value:%u.%u Kpa",(p/1000),(p%1000));
    return 0;
}

short read_bmp280_values16(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned long delay) {
  unsigned char read_buf[2] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  int8_t temp = 0;

//Writes the slave address for write
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_WRITE, 800);

//Writes the pointer to address that needs to be read
  i2c_write_data(i2c_instance, reg_offset , 1000);

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;

//Writes the slave address for read
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_READ, 800);

/* Make a dummy read as per spec of the I2C controller */
  i2c_read_data(i2c_instance, &temp, 1000);

  i2c_read_data(i2c_instance, &read_buf[0], 1000);
  i2c_instance->control = I2C_NACK;
  i2c_read_data(i2c_instance, &read_buf[1], 1000);
  i2c_instance->control = I2C_STOP;

  return ( ( read_buf[1] << 8 ) | read_buf[0] );  
}

int write_bmp280_register(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned char write_value, unsigned long delay) {
  int i = 0, j = 0, k = 0, status=0;
  unsigned int temp = 0;
  i2c_send_slave_address(i2c_instance, BMP280_SLAVE_ADDRESS, I2C_WRITE, 1000);
  i2c_write_data(i2c_instance, reg_offset , 1000);
  i2c_write_data(i2c_instance, write_value , 1000);

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;
  return 0;
}

int init_bmp280() {
  int timeout;
  unsigned int tempReadValue = 0;
  unsigned long pressure = 0, temperature = 0;
  int len;

  write_bmp280_register(I2C, BMP280_CONFIG_REGISTER, 0xC0, 1000);
  write_bmp280_register(I2C, BMP280_CTRL_MEANS, 0x27, 1000);

  if(0 == read_bmp280_register(I2C, 0xD0, &tempReadValue, 1000))
  {
    if (0x58 != tempReadValue)
    {
      printf("BME280 not detected\n");
      return -1;
    }
  }
    
  write_bmp280_register(I2C, BMP280_RESET_REGISTER, 0xB6, 1000);
  read_bmp280_register(I2C, BMP280_RESET_REGISTER, &tempReadValue, 1000);
  
  bmp280_calib_dig_T1 = read_bmp280_values16(I2C, BMP280_REG_DIG_T1, 1000);
  bmp280_calib_dig_T2 = read_bmp280_values16(I2C, BMP280_REG_DIG_T2, 1000);
  bmp280_calib_dig_T3 = read_bmp280_values16(I2C, BMP280_REG_DIG_T3, 1000);

  bmp280_calib_dig_P1 = read_bmp280_values16(I2C, BMP280_REG_DIG_P1, 1000);
  bmp280_calib_dig_P2 = read_bmp280_values16(I2C, BMP280_REG_DIG_P2, 1000);
  bmp280_calib_dig_P3 = read_bmp280_values16(I2C, BMP280_REG_DIG_P3, 1000);
  bmp280_calib_dig_P4 = read_bmp280_values16(I2C, BMP280_REG_DIG_P4, 1000);
  bmp280_calib_dig_P5 = read_bmp280_values16(I2C, BMP280_REG_DIG_P5, 1000);
  bmp280_calib_dig_P6 = read_bmp280_values16(I2C, BMP280_REG_DIG_P6, 1000);
  bmp280_calib_dig_P7 = read_bmp280_values16(I2C, BMP280_REG_DIG_P7, 1000);
  bmp280_calib_dig_P8 = read_bmp280_values16(I2C, BMP280_REG_DIG_P8, 1000);
  bmp280_calib_dig_P9 = read_bmp280_values16(I2C, BMP280_REG_DIG_P9, 1000);
  return 0;
}
