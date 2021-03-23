#include "uart.h" //Includes the definitions of uart communication protocol//
#include "string.h"
#include "pinmux.h"
#include "log.h"

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
  set_baud_rate(SoftSerial, 115200); // baudrate = 9600;
  delay_loop(1000, 1000);

  write_to_uart("Hello World\n");

  initDS3231();
  // updateDS3231Time( 19, 35, 00, 21, 03, 2021);

  while (1)
  {

    // read command from GS ( in 16 byte pieces )
    // this do read non blocking. Not implemented in uart library
    // \n is end of line marker
    // read data from uart. 16 or less chars
    for (uint8_t i=0; i< 200; i++) { data[i] = 0x00; }
    returnCode = read_from_uart(data);

    // \n found, end of command
    if ( returnCode == OK ){ 
      printf("Command for shakti: %s\n",data);
    }

    // no \n found so probably bad data
    if ( returnCode == OVERFLOW ){ 
      printf("OVERFLOW: <%s>\n",data);
    }

    // if we are not receiving a command, then get data for downlink
    getDataFromMPU6050();
    getDataFromHMC5883();
    getDataFromBME280();

    readDS3231(&read_buf);
    sprintf(buf, "OnboardTime : %x-%x-20%x %x:%x:%x\n",
        read_buf[4], read_buf[5], read_buf[6], read_buf[2], read_buf[1], read_buf[0]);
  
    //sprintf(buf, "Msg num: %d\n", cnt);
    write_to_uart(buf);
    printf(buf);
    for (int i=0; i< 16; i++) { buf[i] = 0x00; }
    cnt++;
    delay_loop(3000, 3000);
  }
}
