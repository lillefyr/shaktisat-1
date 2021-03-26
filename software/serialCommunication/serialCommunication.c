#include "uart.h" //Includes the definitions of uart communication protocol//
#include "string.h"
#include "pinmux.h"
#include "log.h"

#include "common.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "BME280.h"
#include "DS3231.h"

#define OK 0
#define OVERFLOW -1

#define SoftSerial uart_instance[1]

char data[200];
char buf[200];
unsigned int read_buf[7] = {0x00};
unsigned int hour, minute, second, date, month, year;
char ch;
int  cnt=0;
int  returnCode;

int8_t bmp280Available = -1;
int8_t bme280Available = -1;

unsigned int tempReadValue = 0;
unsigned long pressure = 0, temperature = 0;


//  Gyro
char mpu6050buffer[20];
int AccX, AccY, AccZ;
int accAngleX, accAngleY, GyroAngleX, GyroAngleY, GyroAngleZ;
int AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int AccErrorX,AccErrory;
int GyroX, GyroY, GyroZ;

int write_to_uart(char *data)
{
  printf(data);
  flush_uart(SoftSerial);
  while (*data != '\n')
  {
    write_uart_character(SoftSerial, *data);
    data++;
  }
  write_uart_character(SoftSerial, '\n');
}

int read_from_uart(char *data) {
  char ch;
  uint8_t retries=0;
  char *str = data;
  for (int i = 0; i < 199; i++)
  {
    // blocking read. We will get a kick from GS every minute
    // should really be unblocking read, but it is not implemented in the library
    // uart buffer contains 16 char
    read_uart_character(SoftSerial, &ch);

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

void main()
{
  *pinmux_config_reg = 0x05;

  // set GPIO2 to interrupt pin
  // set GPIO3 to digital out for LED
  // attachInterrupt(digitalPinToInterrupt(2), setFlag, CHANGE);

  set_baud_rate(SoftSerial, 115200); // baudrate = 9600;
  delay_loop(1000, 1000);

  // I2C init for all devices (except soft I2C)
  i2c_init();

  //Initialises I2C Controller
  if(config_i2c(I2C, PRESCALER_COUNT,SCLK_COUNT))
  {
    printf("Error in I2c initialization, stopping\n");
    return -1;
  }
  printf("I2C Intilized\n");

  write_to_uart("Hello Shaktisat\n");

  // updateDS3231Time( 19, 35, 00, 21, 03, 2021);

  bmp280Available = init_bmp280();
  //bme280Available = init_bme280();

  setAlarmEveryMinute(); // not yet working

  while (1)
  {

    // read command from GS ( in 16 byte pieces )
    // this do read non blocking. Not implemented in uart library
    // \n is end of line marker
    // read data from uart. 16 or less chars
    for (uint8_t i=0; i< 200; i++) { data[i] = 0x00; }
    //returnCode = read_from_uart(data);
    returnCode = 123;

    // \n found, end of command
    if ( returnCode == OK ){ 
      printf("Command for shakti: %s\n",data);
    }

    // no \n found so probably bad data
    if ( returnCode == OVERFLOW ){ 
      printf("OVERFLOW: <%s>\n",data);
    }

    // if we are not receiving a command, then get data for downlink
    getDataFromHMC5883();

    printf("read date and time\n");
    readDS3231(&read_buf);
    sprintf(buf, "OnboardTime : %x-%x-20%x %x:%x:%x\n",
        read_buf[4], read_buf[5], read_buf[6], read_buf[2], read_buf[1], read_buf[0]);

    write_to_uart(buf);
    printf(buf);
    for (int i=0; i< 16; i++) { buf[i] = 0x00; }
    delay_loop(3000, 3000);

    if (bmp280Available == 0 ) {
      write_bmp280_register(I2C, BMP280_CTRL_MEANS, BMP280_NORMAL_MODE, 1000);     // Set it to NORMAL MODE
      if(read_bmp280_register(I2C, BMP280_STATUS_REGISTER, &tempReadValue, 1000) == 0) {
        if(!(tempReadValue & 0x9)) {
          //Read pressure and temperature values.
          read_bmp280_values(I2C, 0xF7, &pressure, &temperature, 1000);

          sprintf(buf, "Pressure : %d, Temperature %d\n", pressure, temperature);
          write_to_uart(buf);
          printf(buf);
          for (int i=0; i< 16; i++) { buf[i] = 0x00; }
        }
      }
      else
      {
        //Display the error
        printf("Temperature read failed.\n");
      }
    }
 
    mpu6050_measuring_value(&mpu6050buffer);
    //printf("AccErrorX: %d\n",AccErrorX);
    //printf("AccErrorY: %d\n",AccErrorY);
    //printf("GyroErrorX: %d\n",GyroErrorX);
    //printf("GyroErrorY: %d\n",GyroErrorY);
    //printf("GyroErrorZ %d\r",GyroErrorZ); 

    printf("AccX: %x\n", (mpu6050buffer[0]<<8 |mpu6050buffer[1])  );
    printf("AccY: %x\n", (mpu6050buffer[2]<<8 |mpu6050buffer[3])  );
    printf("AccZ: %x\n", (mpu6050buffer[4]<<8 |mpu6050buffer[5])  );

    printf("GyroX: %x\n", (mpu6050buffer[8]<<8 |mpu6050buffer[9]) );
    printf("GyroY: %x\n", (mpu6050buffer[10]<<8 |mpu6050buffer[11]) );
    printf("GyroZ: %x\n", (mpu6050buffer[12]<<8 |mpu6050buffer[13]) );

    AccX = (mpu6050buffer[0]<<8 |mpu6050buffer[1]) / 4096; //16-bit X-axis data
    AccY = (mpu6050buffer[2]<<8 |mpu6050buffer[3]) / 4096; //16-bit Y-axis data
    AccZ = (mpu6050buffer[4]<<8 |mpu6050buffer[5]) / 4096; //16-bit Z-axis data

    GyroX = (mpu6050buffer[8]<<8 |mpu6050buffer[9]) / 65.5 ;
    GyroY = (mpu6050buffer[10]<<8 |mpu6050buffer[11]) / 65.5;
    GyroZ = (mpu6050buffer[12]<<8 |mpu6050buffer[13]) / 65.5;

    // Correct the outputs with the calculated error values
    GyroX = GyroX + GyroErrorX ;   // GyroErrorX
    GyroY = GyroY - GyroErrorY;    // GyroErrorY
    GyroZ = GyroZ + GyroErrorZ;    // GyroErrorZ


    //sprintf(buf, "Msg num: %d\n", cnt);


    cnt++;
    delay_loop(3000, 3000);
  }
}
