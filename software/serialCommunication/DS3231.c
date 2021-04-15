#include "i2c.h"
#include "log.h"
#include "uart.h"

#include "common.h"
#include "DS3231.h"

int read_ds3231_registers(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned int *readTemp, unsigned char length, unsigned long delay) {
  unsigned char read_buf[4] = {'\0'};
  int i = 0, j = 0,  k = 0, status=0;
  unsigned char temp = 0;
  int ret;

//Writes the slave address for write
  ret = i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_WRITE, 800);
  if ( ret != 0 ) {
    printf("DS3231: send to slave address failed 0x%x return = %d\n", DS3231_SLAVE_ADDRESS, ret);
    return ret;
  }

//Writes the pointer to address that needs to be read
  ret = i2c_write_data(i2c_instance, reg_offset, delay);
  if ( ret != 0 ) { return ret; }

//Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;

//Writes the slave address for read
  ret = i2c_send_slave_address(i2c_instance, DS3231_SLAVE_ADDRESS, I2C_READ, 800);
  if ( ret != 0 ) { return ret; }

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

int ds3231_init(){
  printf("ds3231_init\n");

  i2c_init();

  if(config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT)) {
    log_error("\tSomething Wrong In Initialization\n");
    return -1;
  }
  else
    printf("\tIntilization done\n");
  return 0;
}


//read the date and return it in read_buf
int readDS3231(char * read_buf){
  return read_ds3231_registers(I2C, DS3231_REG_OFFSET, &read_buf[0], 7, 1000);
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

  return write_ds3231_registers(I2C, 0x00, &write_buf[0], length, 1000);
}


//====== NEW stuff============

void setAlarmEveryMinute() {

// alarm every minute
  unsigned int read_buf[7];
  unsigned int alarm2int;
  unsigned int control;
  unsigned int status;

  read_ds3231_registers(I2C, DS3231_REG_OFFSET, &read_buf[0], 7, 1000);

  //read_buf[0] //second
  //read_buf[1] //minute
  //read_buf[2] //hour
  //read_buf[4] //date
  //read_buf[5] //month
  //read_buf[6] //(year % 100) 
  
  read_buf[0] = 0x59;
  printf ("Alarm is being set to Hour %x Minute %x Second %x\n", read_buf[2], read_buf[1], read_buf[0]);

  write_ds3231_registers(I2C, DS3231M_CONTROL, &read_buf[0], 1, 1000);
  write_ds3231_registers(I2C, DS3231M_CONTROL, &read_buf[1], 1, 1000);
  write_ds3231_registers(I2C, DS3231M_CONTROL, &read_buf[2], 1, 1000);

  read_ds3231_registers(I2C, DS3231M_ALM2MIN, &alarm2int, 1, 1000);
  alarm2int |= 0x80; // alarmType == everyMinute
  write_ds3231_registers(I2C, DS3231M_ALM2MIN, &alarm2int, 1, 1000);

  read_ds3231_registers(I2C, DS3231M_CONTROL, &control, 1, 1000);
  control = control | 0x02; // Set A2IE enable to on
  write_ds3231_registers(I2C, DS3231M_CONTROL, &control, 1, 1000);

  read_ds3231_registers(I2C, DS3231M_STATUS, &status, 1, 1000);
  status = status | 0xFC; // Set A1IE, A2IE enable to off
  write_ds3231_registers(I2C, DS3231M_STATUS, &status, 1, 1000);

  return;
}  // of method setAlarm

void pinAlarm() {
  uint8_t control;
  read_ds3231_registers(I2C, DS3231M_CONTROL, &control, 1, 1000);
  control = control | 0x04; // set bit 3 on
  write_ds3231_registers(I2C, DS3231M_CONTROL, &control, 1, 1000);
}
