#include "traps.h"
#include "clint_driver.h"
#include "uart.h"
#include "string.h"
#include "pinmux.h"
#include "log.h"
#include "gpio_i2c.h"
#include "xadc_driver.h"

#include "common.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "BMP280.h"
#include "BMP280_softI2C.h"
#include "DS3231.h"

#define OK 0
#define OVERFLOW -1

#define HC12_UART uart_instance[1] // HC 12 RX TX, GPIO 0,1
#define GPS_UART uart_instance[2] // GPS RX TX GPIO 2, 3

char uplinkedCommand[200];
char downlinkData[200];
//char ch;
int  cnt=0;
int  commandLength;
uint64_t timerValue;
uint64_t oldTimerValue = 99999;

int8_t ds3231Available = -1;
int8_t bmp280Available = -1;
int8_t bmp280softI2CAvailable = -1;
int8_t hmc5883Available = -1;
int8_t mpu6050Available = -1;

// DS3231
//
unsigned int read_buf[7] = {0x00};
unsigned int tempReadValue = 0;
unsigned long pressure = 0, temperature = 0;

//  Gyro
char readbuf[20];
int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

int shaktiCommand = -1;
int nextShaktiCommand = 0;

int16_t msgcnt = 0;

int write_to_uart(char *data) {
  printf(data);
  flush_uart(HC12_UART);
  int i = 0;
  while (*data != '\n') {
    write_uart_character(HC12_UART, *data);
    data++;
  }
  write_uart_character(HC12_UART, '<');
  write_uart_character(HC12_UART, '/');
  write_uart_character(HC12_UART, '>');
  write_uart_character(HC12_UART, '\n');
}

int read_from_uart(uart_struct *UART, char *data) {
  char ch;
  uint8_t retries=0;
  char *str = data;
  for (int i = 0; i < 199; i++)
  {
    // blocking read. We will get a kick from GS every minute
    // should really be unblocking read, but it is not implemented in the library
    // uart buffer contains 16 char
    read_uart_character(UART, &ch);

    if ( ch == 0x00 ) { ch = '\n'; }

    if ((( ch >= 0x20 ) && ( ch < 0x7F )) || ( ch == '\n' )) {
      if ( ch == '\n' ) { return i; }
      //printf("%c ",ch);
      *str = ch;
      str++;
    }    
  }
  return OVERFLOW;
}


//////// GPS ////////////////////////
char GPGGA[127] = {0}; // $GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18
char GLGSV[127] = {0}; // $GLGSV,1,1,00*65
char GNGLL[127] = {0}; // $GNGLL,,,,,,V,N*i
char GNRMC[127] = {0}; // $GNRMC,,V,,,,,,,,,,N*4D
char GNVTG[127] = {0}; // $GNVTG,,,,,,,,,N,,,09,07,,,12*71

int sendGPSData() {
  uint8_t i;
  char GPSdata[127];
  for (uint8_t i=0; i< 127; i++) { GPSdata[i] = 0x00; }
  printf("read from GPS UART\n");
  int commandLength = read_from_uart(GPS_UART, &GPSdata);

  if (( GPSdata[0] == '$' ) && ( GPSdata[1] == 'G' )) {
    if (( GPSdata[2] == 'P' ) && ( GPSdata[3] == 'G' ) && ( GPSdata[4] == 'G' ) && ( GPSdata[5] == 'A' )){
      for (i=0; ((i < commandLength) && (i < 100)); i++)   { GPGGA[i] = GPSdata[i]; }
    }else{
      if (( GPSdata[2] == 'L' ) && ( GPSdata[3] == 'G' ) && ( GPSdata[4] == 'S' ) && ( GPSdata[5] == 'V' )){
        for (i=0; ((i < commandLength) && (i < 100)); i++)   { GLGSV[i] = GPSdata[i]; }
      }else{
        if ( GPSdata[2] == 'N' ) {
          if (( GPSdata[3] == 'G' ) && ( GPSdata[4] == 'G' ) && ( GPSdata[5] == 'L' )){
            for (i=0; ((i < commandLength) && (i < 100)); i++) { GNGLL[i] = GPSdata[i]; }
          }
          if (( GPSdata[3] == 'R' ) && ( GPSdata[4] == 'M' ) && ( GPSdata[5] == 'C' )){
            for (i=0; ((i < commandLength) && (i < 100)); i++) { GNRMC[i] = GPSdata[i]; }
          }
          if (( GPSdata[3] == 'V' ) && ( GPSdata[4] == 'T' ) && ( GPSdata[5] == 'G' )){
            for (i=0; ((i < commandLength) && (i < 100)); i++) { GNVTG[i] = GPSdata[i]; }
          }
        }else{
          // \n found, end of command
          if ( commandLength > 0 ){
            //downlink "unhandled command"
            sprintf(downlinkData, "ID:30;%s\n",GPSdata);
          }else{
            sprintf(downlinkData, "ID:30;NO GPS data received\n");
          }
          write_to_uart(downlinkData);
	}
      }
    }  
    // At this point we can downlink the data we have
    if ( GPGGA[0] != 0x0 ) { write_to_uart(GPGGA); }
    if ( GLGSV[0] != 0x0 ) { write_to_uart(GLGSV); }
    if ( GNGLL[0] != 0x0 ) { write_to_uart(GNGLL); }
    if ( GNRMC[0] != 0x0 ) { write_to_uart(GNRMC); }
    if ( GNVTG[0] != 0x0 ) { write_to_uart(GNVTG); }
  }else{
    sprintf(downlinkData, "ID:30;Invalid GPS data received (ignore)\n");
  }
}
//////// GPS ////////////////////////

//////// DS3231 SET TIME ////////////////
void setDS3231Time(){
  // ID:11;yy,mm,dd,hh,MM,ss
  // yy year (2 digit) 6,7
  // mm month          9,10
  // dd day            12,13
  // hh hour           15,16
  // MM minute         18,19
  // ss second         21,22

  unsigned int year =   ((uplinkedCommand[6]  - 0x30) * 0x10) + uplinkedCommand[7] - 0x30;
  unsigned int month =  ((uplinkedCommand[9]  - 0x30) * 0x10) + uplinkedCommand[10] - 0x30;
  unsigned int day =    ((uplinkedCommand[12] - 0x30) * 0x10) + uplinkedCommand[13] - 0x30;
  unsigned int hour =   ((uplinkedCommand[15] - 0x30) * 0x10) + uplinkedCommand[16] - 0x30;
  unsigned int minute = ((uplinkedCommand[18] - 0x30) * 0x10) + uplinkedCommand[19] - 0x30;
  unsigned int second = ((uplinkedCommand[21] - 0x30) * 0x10) + uplinkedCommand[22] - 0x30;

  printf("Date set to 20%d/%d/%d %d:%d:%d\n", year, month, day, hour, minute, second);
  updateDS3231Time( hour, minute, second, day, month, year);
}
//////// DS3231 SET TIME ////////////////


//////// QMC5833 / HMC5833 //////////////
void sendHMC5883Data(){
  float heading;
  if ( getDataFromHMC5883(I2C, &heading, 1000) != 0) {
     sprintf(downlinkData, "ID:30;HMC5883 error in heading\n");
  }
  else
  {
    char *headingSign = (heading < 0) ? "-" : "";
    float headingVal = (heading < 0) ? -heading : heading;
    int headingInt1 = headingVal;                  // Get the integer part
    float headingFrac = headingVal - headingInt1;  // Get fraction part
    int headingInt2 = trunc(headingFrac * 10000);  // Turn into integer
    // Print as parts, note that you need 0-padding for fractional bit.

    // When arrives downlink will be converted back to float
    for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
    sprintf (downlinkData, "ID:24;%s%d.%04d\n", headingSign, headingInt1, headingInt2);
    printf(downlinkData);
  }

  write_to_uart(downlinkData); 
}
//////// QMC5833 / HMC5833 //////////////

//////// BoardTemperature //////////////
void sendBoardTemperature(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }

  float temp = xadc_onchip_temp(xadc_read_data(0x41200));
  float voltage = xadc_onchip_voltage(xadc_read_data(0x41204));

  char *tempSign = (temp < 0) ? "-" : "";
  float tempVal = (temp < 0) ? -temp : temp;
  int tempInt1 = tempVal;                  // Get the integer part
  float tempFrac = tempVal - tempInt1;      // Get fraction part
  int tempInt2 = trunc(tempFrac * 10000);  // Turn into integer
  // Print as parts, note that you need 0-padding for fractional bit.

  char *voltSign = (voltage < 0) ? "-" : "";
  float voltVal = (voltage < 0) ? -voltage : voltage;
  int voltInt1 = voltVal;                  // Get the integer part
  float voltFrac = voltVal - voltInt1;      // Get fraction part
  int voltInt2 = trunc(voltFrac * 10000);  // Turn into integer
  // Print as parts, note that you need 0-padding for fractional bit.

  // When arrives downlink will be converted back to float
  sprintf (downlinkData, "ID:25;%s%d.%04d,%s%d.%04d\n", tempSign, tempInt1, tempInt2, voltSign, voltInt1, voltInt2);

  write_to_uart(downlinkData);
}
//////// BoardTemperature //////////////

//////// Status //////////////
void sendStatus(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:30;status not implemented\n");
  write_to_uart(downlinkData);
}
//////// BoardTemperature //////////////

//////// Send PONG //////////////////////
void sendPONG(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:20;PONG\n");
  write_to_uart(downlinkData);  // downlink PONG
}
//////// Send PONG //////////////////////


//////// DS3231 /////////////////////////
void sendDS3231Time(){
  printf("readD3231\n");
  if (ds3231Available != 0 ) {
    // try to initialize again
    ds3231Available = ds3231_init();
  }
  if ( readDS3231(&read_buf) < 0 ) {
    sprintf(downlinkData, "ID:30;DS3231 error\n");
    write_to_uart(downlinkData);  // downlink onboard date and time
    ds3231Available = ds3231_init();
    return;
  } // bad data, skip

  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:21;%x,%x,%x,%x,%x,%x\n",
        read_buf[6], read_buf[5], read_buf[4], read_buf[2], read_buf[1], read_buf[0]);

  if ((read_buf[6] == 0xFF) 
  || (read_buf[5] == 0xFF) 
  || (read_buf[4] == 0xFF) 
  || (read_buf[2] == 0xFF) 
  || (read_buf[1] == 0xFF) 
  || (read_buf[0] == 0xFF)){
    // bad data, dont send
    sprintf(downlinkData, "ID:30;DS3231 error\n");
    return;
  }

  write_to_uart(downlinkData);  // downlink onboard date and time
}
//////// DS3231 /////////////////////////

//////// BMP280 /////////////////////////
void sendBMP280Data(){
  printf("read pressure and temp\n");
  if (bmp280Available == 1 ) {
    // try to initialize again
      bmp280Available = bmp280_init();
  }

  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  if (bmp280Available == 0 ) {
    write_bmp280_register(I2C, BMP280_CTRL_MEANS, BMP280_NORMAL_MODE, 1000);     // Set it to NORMAL MODE
    if(read_bmp280_register(I2C, BMP280_STATUS_REGISTER, &tempReadValue, 1000) == 0) {
      if(!(tempReadValue & 0x9)) {
        //Read pressure and temperature values.
        read_bmp280_values(I2C, 0xF7, &pressure, &temperature, 1000);

        // set decimal point  hectoPascal and 2 decimals on temp
        sprintf(downlinkData, "ID:22;%d.%d,%d.%d\n", pressure/100, pressure%100, temperature/100, temperature%100);
      }
      else
      {
        sprintf(downlinkData, "ID:30;BMP280 Temperature invalid\n");
        bmp280Available = 1; // try to reconnect again
      }
    }
    else
    {
      //Display the error
      sprintf(downlinkData, "ID:30;BMP280 I2C bus nack\n");
      bmp280Available = 1; // try to reconnect again
    }
  }
  else
  {
    //Display the error
    sprintf(downlinkData, "ID:30;BMP280 not available\n");
    bmp280Available = 1; // try to reconnect again
  }
  write_to_uart(downlinkData);
}
//////// BMP280 /////////////////////////
//
//////// BMP280_softI2C /////////////////////////
void sendBMP280_softI2C_Data(){
  int i;
  bmp280Available = 1;
  printf("bmp280_softI2C  read pressure and temp\n");
  if (bmp280Available == 1 ) {
    // try to initialize again
    bmp280softI2CAvailable = bmp280_softI2C_init();
  }

  for (i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  if ( bmp280softI2CAvailable == 0 ) {
    write_bmp280_softI2C_register(BMP280_CTRL_MEANS, BMP280_NORMAL_MODE, 1000);     // Set it to NORMAL MODE

    tempReadValue = 0x09; // zero is true
    i = 0;
    while ((tempReadValue & 0x09) && (i<32000)) {
      read_bmp280_softI2C_register(BMP280_STATUS_REGISTER, &tempReadValue, 1000);
      i++;
    }
    if ( i >= 32000 ) {
      //Display the error
      sprintf(downlinkData, "ID:30;bmp280_softI2C not ready\n");
      bmp280softI2CAvailable = 1; // try to reconnect again
    }
    else
    {
      //Read pressure and temperature values.
      read_bmp280_softI2C_values(0xF7, &pressure, &temperature, 1000);
    }
  }
  else
  {
    //Display the error
    sprintf(downlinkData, "ID:30;bmp280_softI2C not available\n");
    bmp280softI2CAvailable = 1; // try to reconnect again
  }
  write_to_uart(downlinkData);
}
//////// BMP280_softI2C /////////////////////////
 
 
//////// MPU6050 /////////////////////////
void sendMPU6050Data(){
  printf("Read acceleration and gyro\n");
  if ( mpu6050Available == 1 ){
    // try to initialize again
    mpu6050Available = mpu6050_init();
  }

  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  if ( mpu6050Available == 0 ){
    mpu6050_measuring_value(&readbuf);
    AccX = (readbuf[0]<<8 |readbuf[1]);
    AccY = (readbuf[2]<<8 |readbuf[3]);
    AccZ = (readbuf[4]<<8 |readbuf[5]);
    GyroX = (readbuf[8]<<8 |readbuf[9]);
    GyroY = (readbuf[10]<<8 |readbuf[11]);
    GyroZ = (readbuf[12]<<8 |readbuf[13]);

    sprintf(downlinkData, "ID:23;%d,%d,%d,%d,%d,%d\n", AccX, AccY, AccZ, GyroX, GyroY, GyroZ);
  }
  else
  {
    sprintf(downlinkData, "ID:30;MPU6050 failure\n");
  }
  write_to_uart(downlinkData);
}
//////// MPU6050 /////////////////////////

uint8_t cnt1 = 0;

int runInit(){
  *pinmux_config_reg = 0x0055;

  set_baud_rate(HC12_UART, 9600);
  set_baud_rate(GPS_UART, 9600);

  delay_loop(1000, 1000);

  // I2C init for all devices (except soft I2C)
  //printf("i2c init: SCL is %x and expected %x, SDA is %x and expected %x\n", I2C_SCL, GPIO5, I2C_SDA, GPIO6);
  //printf("change in bsp/include/gpio_i2c.h\n");
  i2c_init();

  //Initialises I2C Controller
  if(config_i2c(I2C, PRESCALER_COUNT,SCLK_COUNT)) {
    if ( cnt1 == 0 ){
      sprintf(downlinkData, "ID:30;Error in I2c initialization\n");
      write_to_uart(downlinkData);
    }
    cnt1++;
    return -1;
  }
  printf("I2C Intilized\n");

  // updateDS3231Time( 19, 35, 00, 21, 03, 2021);

  ds3231Available = ds3231_init();
  bmp280Available = bmp280_init();
  bmp280softI2CAvailable = bmp280_softI2C_init();
  mpu6050Available = mpu6050_init();
  hmc5883Available = hmc5883_init(); 

  return 0;
}

/////// MAIN ////////////////////////////
void main() {

  printf("ShaktiSat1 SerialCommunication\n");
  printf("\n");
  sprintf(downlinkData, "ID:30;ShaktiSat1 starting\n");
  write_to_uart(downlinkData);
  delay_loop(1000, 1000);

  int rc = 1;
  while (rc != 0) {
    rc = runInit();
  }

  sprintf(downlinkData, "ID:30;ShaktiSat1 online\n");
  write_to_uart(downlinkData);
  delay_loop(1000, 1000);

  nextShaktiCommand = 0;

  while (1)
  {
    // read command from GS ( in 16 byte pieces )
    // this do read non blocking. Not implemented in uart library
    // \n is end of line marker
    // read uplinkedCommand from uart. 16 or less chars
    for (uint8_t i=0; i< 200; i++) { uplinkedCommand[i] = 0x00; }
    commandLength = read_from_uart(HC12_UART, uplinkedCommand);

    // \n found, end of command
    if ( commandLength > 0 ){ 
      printf("Command for shakti: %s\n",uplinkedCommand);
      if ((uplinkedCommand[0] == 'I') && (uplinkedCommand[1] == 'D') && (uplinkedCommand[2] == ':')){
         shaktiCommand = ((uplinkedCommand[3]  - 0x30) * 0x10) + uplinkedCommand[4] - 0x30;
	 printf("shakti command 0x%x\n", shaktiCommand);
      }
    }

    // no \n found so probably bad uplinkedCommand
    if ( commandLength == OVERFLOW ){ 
      printf("OVERFLOW: <%s>\n",uplinkedCommand);
    }

    // get data for downlink
    // This is approx 10 seconds
    timerValue = get_timer_value()/200000000;
    if (oldTimerValue != timerValue) {
      printf("timervalue=%d ", timerValue);
      printf("oldTimerValue=%d ", oldTimerValue);
      printf("nextShaktiCommand=%d ", nextShaktiCommand);
      printf("shaktiCommand=%d \n", shaktiCommand);
      shaktiCommand = nextShaktiCommand;
      oldTimerValue = timerValue;
    }

    switch ( shaktiCommand ){
      case 0x00:
        sendPONG();
        nextShaktiCommand=1;
        break;

      case 0x01:
        sendDS3231Time();
        nextShaktiCommand=2;
        break;

      case 0x02:
        sendBMP280Data();
	// In case of error, retry once
        if (bmp280Available == 1 ){
           sendBMP280Data();
        }
        nextShaktiCommand=3;
        break;

      case 0x03:
        sendMPU6050Data();
        nextShaktiCommand=4;
        break;

      case 0x04:
        sendHMC5883Data();
        nextShaktiCommand=5;
        break;

      case 0x05:
        sendBoardTemperature();
        nextShaktiCommand = 9;
        break;

      case 0x09:
        sendGPSData();
        nextShaktiCommand = 0x10;
        break;

      case 0x10:
        sendStatus();
        nextShaktiCommand = 0;
        break;

      case 0x11:
        setDS3231Time();
        break;

      case 0x12:
        sendBMP280_softI2C_Data();
        break;

      case 0x99:
        sprintf(downlinkData, "ID:30;ShaktiSat1 restarting\n");
        write_to_uart(downlinkData);
        rc = runInit();
	if ( rc != 0 ) {
          sprintf(downlinkData, "ID:30;Init failed. rc = %d\n", rc);
	} else {
          sprintf(downlinkData, "ID:30;ShaktiSat1 restartet\n");
	}
        write_to_uart(downlinkData);
	break;

      default:
        break;
    }

//    delay_loop(3000, 3000);
//    sprintf(downlinkData, "ID:30;Shaktisat %d\n", msgcnt);
//    write_to_uart(downlinkData);
//    msgcnt++;
    shaktiCommand = -nextShaktiCommand;
  }
}
