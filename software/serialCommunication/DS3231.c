
#include "i2c.h"
#include "log.h"
#include "uart.h"

#define UPDATE_TIME 1
#define I2C i2c_instance[1]
#define DS3231_SLAVE_ADDRESS 0XD0
#define DS3231_REG_OFFSET 0
#define DS3231_DEC_TO_HEX(decimal)  ( ( (decimal / 10 ) << 4) | (decimal % 10) )
#define DELAY_VALUE 900
#define PRESCALER_COUNT 0x1F
#define SCLK_COUNT 0x91

int read_ds3231_registers(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned int *readTemp, unsigned char length, unsigned long delay)
{
  unsigned char read_buf[4] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  unsigned char temp = 0;
  int ret;

//Writes the slave address for write
  ret = i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_WRITE, 800);
//  printf("send slave address 0x%x return = %d\n", DS3231_SLAVE_ADDRESS, ret);

//Writes the pointer to address that needs to be read
  ret = i2c_write_data(i2c_instance, reg_offset, delay);

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;

//Writes the slave address for read
  ret = i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_READ, 800);

  temp = 0;
/* Make a dummy read as per spec of the I2C controller */
  ret = i2c_read_data(i2c_instance, &temp, delay);

//Reads the MSB Byte of temperature [D9 - D1]
  for(i = 0; i < length; i++)
  {
    ret = i2c_read_data(i2c_instance, &temp, delay);
    *readTemp = temp;
    if( i == (length - 1) ){
      i2c_instance->control = I2C_NACK;
    }
    *readTemp++;
  }
  i2c_instance->control = I2C_STOP;
  return 0;
}

int write_ds3231_registers(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned int *write_value, unsigned char length, unsigned long delay)
{
  //int i = 0, j = 0,  k = 0, status=0;
  //unsigned int temp = 0;
  i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_WRITE, delay);
  i2c_write_data(i2c_instance, reg_offset, delay);
  for(int i = 0; i < length; i++)
  {
    i2c_write_data(i2c_instance,  ( *write_value++  & 0xff) /*LM75_TEMP_REG_OFFSET */, delay);
  }
  i2c_instance->control = I2C_STOP;
  return 0;
}

int initDS3231(){

  i2c_init();

  if(config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT))
  {
    log_error("\tSomething Wrong In Initialization\n");
    return -1;
  }
  else
    printf("\tIntilization done\n");
  return 0;
}


//read the date and return it in read_buf
int readDS3231(char * read_buf){
  read_ds3231_registers(I2C, DS3231_REG_OFFSET, &read_buf[0], 7, 1000);
};

//set the time on DS3231
int updateDS3231Time( unsigned int hour, unsigned int minute, unsigned int second,
                      unsigned int date, unsigned int month, unsigned int year){
  unsigned char length;
  unsigned int write_buf[7];
//  hour = 9;
//  minute = 0;
//  second = 0;

//  date = 18;
//  month = 03;
//  year = 2021;

  write_buf[0] = DS3231_DEC_TO_HEX(second);
  write_buf[1] = DS3231_DEC_TO_HEX(minute);
  write_buf[2] = DS3231_DEC_TO_HEX(hour);
  write_buf[4] = DS3231_DEC_TO_HEX(date);
  write_buf[5] = DS3231_DEC_TO_HEX(month);
  write_buf[6] = DS3231_DEC_TO_HEX((year % 100));
  length = 7;

  write_ds3231_registers(I2C, 0x00, &write_buf[0], length, 1000);
}
