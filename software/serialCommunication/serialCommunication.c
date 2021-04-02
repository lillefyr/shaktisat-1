#include "uart.h" //Includes the definitions of uart communication protocol//
#include "string.h"
#include "pinmux.h"
#include "log.h"
#include "gpio_i2c.h"

#include "common.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "BME280.h"
#include "BMP280.h"
#include "DS3231.h"

#define OK 0
#define OVERFLOW -1

#define HC12_UART uart_instance[1] // HC 12 RX TX, GPIO 0,1
#define GPS_UART uart_instance[2] // GPS RX TX GPIO 2, 3

char uplinkedCommand[200];
char downlinkData[200];
unsigned int read_buf[7] = {0x00};
char ch;
int  cnt=0;
int  returnCode;

int8_t bmp280Available = -1;
int8_t bme280Available = -1;
int8_t mpu6050Available = -1;
int8_t hmc5883Available = -1;

unsigned int tempReadValue = 0;
unsigned long pressure = 0, temperature = 0;

//  Gyro
char readbuf[20];
int GyroErrorX, GyroErrorY, GyroErrorZ;
int accAngleX;
int accAngleY;

int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

int16_t averageAccX;
int16_t averageAccY;
int16_t averageAccZ;
int16_t averageGyroX;
int16_t averageGyroY;
int16_t averageGyroZ;

uint8_t firsttime = 0;

int8_t shaktiCommand = -1;

int16_t msgcnt = 0;

int write_to_uart(char *data) {
  printf(data);
  flush_uart(HC12_UART);
  int i = 0;
  while (*data != '\n') {
    write_uart_character(HC12_UART, *data);
    data++;
  }
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
      if ( ch == '\n' ) { return OK; }
      //printf("%c ",ch);
      *str = ch;
      str++;
    }    
  }
  return OVERFLOW;
}


//////// GPS ////////////////////////
int sendGPSData() {
  char GPSdata[100];
  for (uint8_t i=0; i< 200; i++) { GPSdata[i] = 0x00; }
  returnCode = read_from_uart(GPS_UART, &GPSdata);

  // \n found, end of command
  if ( returnCode == OK ){
    printf("GPS data received: %s\n",GPSdata);
  }else{
    printf("NO GPS data received: %s\n",GPSdata);
  }
}
//////// GPS ////////////////////////

//////// DS3231 SET TIME ////////////////
void setDS3231Time(){
  // ID:31;yy,mm,dd,hh,MM,ss
  // yy year (2 digit) 6,7
  // mm month          9,10
  // dd day            12,13
  // hh hour           15,16
  // MM minute         18,19
  // ss second         21,22

  unsigned int year =   ((uplinkedCommand[6]  - 0x30) * 10) + uplinkedCommand[7] - 0x30;
  unsigned int month =  ((uplinkedCommand[9]  - 0x30) * 10) + uplinkedCommand[10] - 0x30;
  unsigned int day =    ((uplinkedCommand[12] - 0x30) * 10) + uplinkedCommand[13] - 0x30;
  unsigned int hour =   ((uplinkedCommand[15] - 0x30) * 10) + uplinkedCommand[16] - 0x30;
  unsigned int minute = ((uplinkedCommand[18] - 0x30) * 10) + uplinkedCommand[19] - 0x30;
  unsigned int second = ((uplinkedCommand[21] - 0x30) * 10) + uplinkedCommand[22] - 0x30;

  updateDS3231Time( hour, minute, second, day, month, year);
  printf("Date set to 20%x/%x/%x %x:%x:%x\n", year, month, day, hour, minute, second);
}
//////// DS3231 SET TIME ////////////////


//////// QMC5833 / HMC5833 //////////////
void sendHMC5883Data(){
  getDataFromHMC5883();
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:30;HMC5883 not implemented\n");
  write_to_uart(downlinkData); 
  printf(downlinkData);
}
//////// QMC5833 / HMC5833 //////////////

//////// BoardTemperature //////////////
void sendBoardTemperature(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:30;Boardtemp not implemented\n"); // max length of message!!!
  write_to_uart(downlinkData);
  printf(downlinkData);
}
//////// BoardTemperature //////////////

//////// Status //////////////
void sendStatus(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:30;status not implemented\n");
  write_to_uart(downlinkData);
  printf(downlinkData);
}
//////// BoardTemperature //////////////

//////// Send PONG //////////////////////
void sendPONG(){
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:20;PONG\n");
  write_to_uart(downlinkData);  // downlink PONG
  printf(downlinkData);
}
//////// Send PONG //////////////////////

//////// DS3231 /////////////////////////
void sendDS3231Time(){
  printf("read date and time\n");
  readDS3231(&read_buf);
  for (int i=0; i < 32; i++) { downlinkData[i] = 0x00; }
  sprintf(downlinkData, "ID:21;%x,%x,%x,%x,%x,%x\n",
        read_buf[6], read_buf[5], read_buf[4], read_buf[2], read_buf[1], read_buf[0]);

  write_to_uart(downlinkData);  // downlink date and time
  printf(downlinkData);
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

        sprintf(downlinkData, "ID22;%d,%d\n", pressure, temperature);
      }
      else
      {
        sprintf(downlinkData, "ID:30;BMP280 read failed (1)\n");
        bmp280Available = 1; // try to reconnect again
      }
    }
    else
    {
      //Display the error
      sprintf(downlinkData, "ID:30;BMP280 read failed (2)\n");
      bmp280Available = 1; // try to reconnect again
    }
  }
  else
  {
    //Display the error
    sprintf(downlinkData, "ID:30;BMP280 read failed (3)\n");
    bmp280Available = 1; // try to reconnect again
  }
  write_to_uart(downlinkData);
  printf(downlinkData);
}
//////// BMP280 /////////////////////////
 
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
  printf(downlinkData);
}
//////// MPU6050 /////////////////////////

void main() {
  *pinmux_config_reg = 0x0055;

  // set GPIO2 to interrupt pin
  // set GPIO3 to digital out for LED
  // attachInterrupt(digitalPinToInterrupt(2), setFlag, CHANGE);

  set_baud_rate(HC12_UART, 9600);
  set_baud_rate(GPS_UART, 9600);

  delay_loop(1000, 1000);

  sprintf(downlinkData, "ID:30;Hello from Shakti\n");
  write_to_uart(downlinkData);  // downlink PONG
  delay_loop(1000, 1000);

  // I2C init for all devices (except soft I2C)
  printf("i2c init: SCL is %x and expected %x, SDA is %x and expected %x\n", I2C_SCL, GPIO5, I2C_SDA, GPIO6);
  printf("change in bsp/include/gpio_i2c.h\n");
  i2c_init();

  //Initialises I2C Controller
  if(config_i2c(I2C, PRESCALER_COUNT,SCLK_COUNT)) {
    printf("Error in I2c initialization, stopping\n");
    return -1;
  }
  printf("I2C Intilized\n");

  // updateDS3231Time( 19, 35, 00, 21, 03, 2021);

  bmp280Available = bmp280_init();
  bme280Available = bme280_init(); // no code available yet
  mpu6050Available = mpu6050_init();
  hmc5883Available = hmc5883_init(); // no code available yet

//  setAlarmEveryMinute(); // not yet working

  while (1)
  {
    // read command from GS ( in 16 byte pieces )
    // this do read non blocking. Not implemented in uart library
    // \n is end of line marker
    // read uplinkedCommand from uart. 16 or less chars
    for (uint8_t i=0; i< 200; i++) { uplinkedCommand[i] = 0x00; }
    returnCode = read_from_uart(HC12_UART, uplinkedCommand);

    // \n found, end of command
    shaktiCommand = -1;
    if ( returnCode == OK ){ 
      printf("Command for shakti: %s\n",uplinkedCommand);
      if ((uplinkedCommand[0] == 'I') && (uplinkedCommand[1] == 'D') && (uplinkedCommand[2] == ':')){
         shaktiCommand = ((uplinkedCommand[3]  - 0x30) * 10) + uplinkedCommand[4] - 0x30;
      }
    }

    // no \n found so probably bad uplinkedCommand
    if ( returnCode == OVERFLOW ){ 
      printf("OVERFLOW: <%s>\n",uplinkedCommand);
    }

    // get data for downlink
    // temp entry
    if (( shaktiCommand == -1 ) && ((msgcnt % 10 ) == 0)) {
      shaktiCommand = 1;
    }

    switch ( shaktiCommand ){
      case 0x00:
        sendPONG();
        break;

      case 0x01:
        sendDS3231Time();
        break;

      case 0x02:
        sendBMP280Data();
        break;

      case 0x03:
        sendMPU6050Data();
        break;

      case 0x04:
        sendHMC5883Data();
        break;

      case 0x05:
        sendBoardTemperature();
        break;

      case 0x09:;
        sendGPSData();
        break;

      case 0x10:
        sendStatus();
        break;

      default:
        break;
    }

    delay_loop(3000, 3000);
    sprintf(downlinkData, "ID:30;Shaktisat %d\n", msgcnt);
    write_to_uart(downlinkData);
    msgcnt++;
  }
}
