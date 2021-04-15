#include"gpio_i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1
#include "common.h"
#include "BMP280_softI2C.h"

// https://www.digikey.com/en/datasheets/bosch-sensortec/bosch-sensortec-bst-bmp280-ds001-19#pf18
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

#define BME280_REG_DIG_H1 0xA1
#define BME280_REG_DIG_H2 0xE1
#define BME280_REG_DIG_H3 0xE3
#define BME280_REG_DIG_H4 0xE4
#define BME280_REG_DIG_H5 0xE5
#define BME280_REG_DIG_H6 0xE7

#define BME280_REG_CHIPID 0xD0    //0x58
#define BME280_REG_VERSION 0xD1
#define BME280_REG_SOFTRESET 0xE0
#define BMP280_RESET_REGISTER 0xE0

#define BME280_REG_CAL26 0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REG_CONTROLHUMID 0xF2
#define BME280_REG_STATUS 0XF3
#define BMP280_STATUS_REGISTER 0xF3
#define BME280_REG_CONTROL 0xF4
#define BME280_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BMP280_CONFIG_REGISTER 0xF5
#define BME280_REG_PRESSUREDATA 0xF7 //7,8,9
#define BME280_REG_TEMPDATA 0xFA // a,b,c
#define BME280_REG_HUMIDDATA 0xFD // not for BME280

// extra
#define BMP280_NORMAL_MODE 0x26
#define BMP280_SLAVE_ADDRESS 0xEC  //Defines the Starting address of slave//


// 0xEC 0xA0 register 0xA1 data

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

int read_bmp280_softI2C_register(unsigned int reg_offset, unsigned int *readTemp, unsigned long delay) {
  unsigned char read_buf[4] = {'\0'};
  unsigned char temp = 0;

  printf("Register: 0x%x, target address = 0x%x\n", reg_offset, readTemp);
  I2cSendSlaveAddress( BMP280_SLAVE_ADDRESS, I2C_WRITE, 100);
  I2cWriteData(reg_offset, 100);
  I2cSendSlaveAddress( BMP280_SLAVE_ADDRESS, I2C_READ, 100);

// Make a dummy read as per spec of the I2C controller
  read_buf[0] = I2cReadDataAck(100);
  SendNackForRead(100);

//Reads the MSB Byte of temperature [D9 - D1]
  read_buf[0] = I2cReadDataAck(100);
  read_buf[1] = I2cReadDataNack(100);

  printf("readTemp = 0x%x\n",read_buf[0]);
  *readTemp = read_buf[0] ;
  return 0;
}

int read_bmp280_softI2C_values(unsigned int reg_offset, unsigned long *pressure, unsigned long *temperature, unsigned long delay) {
  unsigned char read_buf[7] = {'\0'};
  int32_t adc_P, adc_T, var1, var2, t_fine, temp;
  int32_t var3, var4, p;
  int ret;
  int asbjorn=0;

  if (asbjorn > 0) {
//Read Pressure
  printf("read 0xF7\n");
  read_bmp280_softI2C_register(0xF7, &read_buf[0], 100); // bit 19:12
  printf("read 0xF8\n");
  read_bmp280_softI2C_register(0xF8, &read_buf[1], 100); // bit 11:4
  printf("read 0xF9\n");
  read_bmp280_softI2C_register(0xF9, &read_buf[2], 100); // bit 3:0 (oversampling)
  
//Read Temperature  
  printf("read 0xFA\n");
  read_bmp280_softI2C_register(0xFA, &read_buf[3], 100);  // bit 19:12
  printf("read 0xFB\n");
  read_bmp280_softI2C_register(0xFB, &read_buf[4], 100);  // bit 11:4
  printf("read 0xFC\n");
  read_bmp280_softI2C_register(0xFC, &read_buf[5], 100);  // bit 3:0 (oversampling) set in 0xF4 to 001
  }
  else
  {
    printf("I2cSendSlaveAddress\n");
    I2cSendSlaveAddress(BMP280_SLAVE_ADDRESS, I2C_WRITE, 100);//selecting slave to be read
    printf("I2cWriteData\n");
    I2cWriteData(0xF7, 100);//selecting register to be read
    printf("I2cSendSlaveAddress\n");
    I2cSendSlaveAddress(BMP280_SLAVE_ADDRESS, I2C_READ, 100);
    printf("I2c_shakti_read_bytes %d\n", 
    I2c_shakti_readbytes(read_buf,6,1, 100));//to read the output values
  }

  printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4], read_buf[5]);
  
  adc_P = ((read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4));
  adc_T = ((read_buf[3] << 12) | (read_buf[4] << 4) | (read_buf[5] >> 4));

  printf("Pressure %x temperature %x .. %d Pascal\n", adc_P, adc_T, (adc_P*2.62));

// Calculate TEMPERATURE (from data sheet)
// 81990   1000 0001 1001 1001 0000
  var1 = ((((adc_T / 8) - ((int32_t)bmp280_calib_dig_T1 * 2))) * ((int32_t)bmp280_calib_dig_T2)) / 2048;
  var2 = (((((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1)) * ((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1))) / 4096) * ((int32_t)bmp280_calib_dig_T3)) / 16384;
  t_fine = var1 + var2;
  temp = (t_fine * 5 + 128) / 256;
  *temperature = temp;
  printf("Temperature Value:%u.%u °C\n", (temp/100),(temp%100));

  for (int64_t i = 0x00008; i< 0x4000000; i=i*0x10)
  {
    adc_T = i;
    var1 = ((((adc_T / 8) - ((int32_t)bmp280_calib_dig_T1 * 2))) * ((int32_t)bmp280_calib_dig_T2)) / 2048;
    var2 = (((((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1)) * ((adc_T / 16) - ((int32_t)bmp280_calib_dig_T1))) / 4096) * ((int32_t)bmp280_calib_dig_T3)) / 16384;
    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) / 256;
    printf("adc_t = 0x%x Temperature %d Value:%d.%d °C\n", adc_T, temp, (temp/100),(temp%100));
  }
	  

//Calculate Pressure
  var3 = (((int64_t)t_fine) / 2) - (int64_t)64000;
  var4 = (((var3/4) * (var3/4)) / 2048 ) * ((int64_t)bmp280_calib_dig_P6);
  var4 = var4 + ((var3 * ((int64_t)bmp280_calib_dig_P5)) * 2);
  var4 = (var4/4) + (((int64_t)bmp280_calib_dig_P4) * 65536);
  var3 = ((((int64_t)bmp280_calib_dig_P3 * (((var3/4) * (var3/4)) / 8192 )) / 8) + ((((int64_t)bmp280_calib_dig_P2) * var3)/2)) / 262144;
  var3 =((((32768 + var3)) * ((int64_t)bmp280_calib_dig_P1)) / 32768);

  if (var3 == 0)
    return 0; // avoid exception caused by division by zero

  p = (((uint64_t)(((int64_t)1048576) - adc_P) - (var4 / 4096))) * 3125;

  if (p < 0x80000000){
    p = (p * 2) / ((uint64_t)var3);
  } 
  else
  {
    p = (p / (uint64_t)var3) * 2;
  }

  var3 = (((int64_t)bmp280_calib_dig_P9) * ((int64_t)(((p/8) * (p/8)) / 8192))) / 4096;
  var4 = (((int64_t)(p/4)) * ((int64_t)bmp280_calib_dig_P8)) / 8192;

  p = (uint64_t)((int64_t)p + ((var3 + var4 + (int64_t)bmp280_calib_dig_P7) / 16));
  *pressure = p;
  printf("The Pressure Value:%u.%u Kpa\n",(p/100),(p%1000));
  return 0;
}

short read_bmp280_softI2C_values16(unsigned int reg_offset, unsigned long delay) {
  unsigned char read_buf[2] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  int8_t temp = 0;
  int ret;

//Writes the slave address for write
  I2cSendSlaveAddress( BMP280_SLAVE_ADDRESS, I2C_WRITE, 800);

//Writes the pointer to address that needs to be read
  I2cWriteData(reg_offset , 100);

//Writes the slave address for read
  I2cSendSlaveAddress( BMP280_SLAVE_ADDRESS, I2C_READ, 800);

// Make a dummy read as per spec of the I2C controller
  I2c_Read_byte(BMP280_SLAVE_ADDRESS, &temp, 100);

  I2c_Read_byte(BMP280_SLAVE_ADDRESS, &read_buf[0], 100);

  SendNackForRead(100);
  I2c_Read_byte(BMP280_SLAVE_ADDRESS, &read_buf[1], 100);

  return ( ( read_buf[1] << 8 ) | read_buf[0] );  
}

//int write_bmp280_register(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned char write_value, unsigned long delay) {
int write_bmp280_softI2C_register(unsigned int reg_offset, unsigned char write_value, unsigned long delay) {
  I2cSendSlaveAddress( BMP280_SLAVE_ADDRESS, I2C_WRITE, 100);

  I2c_Write_byte(BMP280_SLAVE_ADDRESS, reg_offset , write_value , 100);

  return 0;
}

int bmp280_softI2C_init() {
  int timeout;
  unsigned int tempReadValue = 0;
  unsigned long pressure = 0, temperature = 0;
  int len;
  int ret;
  return -1;
  printf("\nbmp280_softI2c_init\n");

  I2cInit();//configuring data and clocks as output lines

  printf("\nBMP280_RESET_REGISTER 0xE0\n");
  // command 0xB6 resets the device
  I2c_Write_byte(BMP280_SLAVE_ADDRESS, BMP280_RESET_REGISTER, 0xB6, 100);

  printf("BME280_REG_CHIPID 0xD0\n");
  tempReadValue = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BME280_REG_CHIPID, 100);
  if (0x58 != tempReadValue) {
    printf("bmp280_softI2C not detected %x\n", tempReadValue);
    return -1;
  }

  printf("\nBMP280_CONFIG_REGISTER 0xF5\n");
  // 0xCO    1100 0000  Inactive duration 0b110, timeconstant 0b000,  ignore bit 1, spi (when 1) 0b0 
  // Standby time  between measurements 2000ms (could be 4000ms with 111)
  // Skip filter settings, disturbances will be measured
  // I2c interface
  I2c_Write_byte(BMP280_SLAVE_ADDRESS, BMP280_CONFIG_REGISTER, 0xC0, 100);

  printf("BME280_REG_CONTROL 0xF4\n");
  // 0x27    0010 0111  Oversampling temp 0b001, oversampling press 0b001, mode 0b11
  // least precise temp, fast measurement
  // least precise press, fast measurement, if 000, the no pressure measured
  // noamal mode 11, could be sleep mode 00
  I2c_Write_byte(BMP280_SLAVE_ADDRESS, BME280_REG_CONTROL, 0x27, 100);
 
// global variables used when reading temp and pressure
  bmp280_calib_dig_T1 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_T1, 100);
  bmp280_calib_dig_T2 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_T2, 100);
  bmp280_calib_dig_T3 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_T3, 100);

  bmp280_calib_dig_P1 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P1, 100);
  bmp280_calib_dig_P2 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P2, 100);
  bmp280_calib_dig_P3 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P3, 100);
  bmp280_calib_dig_P4 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P4, 100);
  bmp280_calib_dig_P5 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P5, 100);
  bmp280_calib_dig_P6 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P6, 100);
  bmp280_calib_dig_P7 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P7, 100);
  bmp280_calib_dig_P8 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P8, 100);
  bmp280_calib_dig_P9 = I2c_Read_byte(BMP280_SLAVE_ADDRESS, BMP280_REG_DIG_P9, 100);

  printf("T1 0x%x\n", bmp280_calib_dig_T1);
  printf("T2 0x%x\n", bmp280_calib_dig_T2);
  printf("T3 0x%x\n", bmp280_calib_dig_T3);

  printf("P1 0x%x\n", bmp280_calib_dig_P1);
  printf("P2 0x%x\n", bmp280_calib_dig_P2);
  printf("P3 0x%x\n", bmp280_calib_dig_P3);
  printf("P4 0x%x\n", bmp280_calib_dig_P4);
  printf("P5 0x%x\n", bmp280_calib_dig_P5);
  printf("P6 0x%x\n", bmp280_calib_dig_P6);
  printf("P7 0x%x\n", bmp280_calib_dig_P7);
  printf("P8 0x%x\n", bmp280_calib_dig_P8);
  printf("P9 0x%x\n", bmp280_calib_dig_P9);

  printf("bmp280_softI2C ready\n");
  return 0;
}
