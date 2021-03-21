#include "uart.h" //Includes the definitions of uart communication protocol//
#include "string.h"
#include "pinmux.h"
#include "log.h"

#include "MPU6050.h"
#include "HMC5883.h"
#include "BME280.h"
#include "DS3231.h"

#define OK 0
#define PART -1
#define EOD -2

#define SoftSerial uart_instance[1]

char data[17]; // only 16 byte i uart
char command[200];
uint8_t command_start = 0;
char ch;
char buf[15];
int  cnt=0;
int  returnCode;
uint32_t checksum;
uint32_t checksum2;


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
  for (int i = 0; i < 16; i++)
  {
    //printf("status=  0x%x  Flag=  0x%x\n", SoftSerial->status, STS_RX_NOT_EMPTY);
    // if no data available return
    if ((SoftSerial->status & STS_RX_NOT_EMPTY) != STS_RX_NOT_EMPTY) { return EOD; }

    //read_uart_character(SoftSerial, &ch);
    ch = SoftSerial->rcv_reg;

    if ((( ch >= 0x20 ) && ( ch < 0x7F )) || ( ch == '\n' )) {
      if ( ch == '\n' ) { printf("data read\n"); return OK; }
      printf("%c ",ch);
      *str = ch;
      str++;
    }    
  }
  return PART;
}

void main()
{
  *pinmux_config_reg = 0x05;
  set_baud_rate(SoftSerial, 115200); // baudrate = 9600;
  delay_loop(1000, 1000);

  write_to_uart("Hello World\n");

  while (1)
  {

    // read command from GS ( in 16 byte pieces )
    // this do read non blocking. Not implemented in uart library
    // \n is end of line marker

    // reset buffer
    for (int i=0; i< 15; i++) { data[i] = '\n'; }

    // read data from uart. 16 or less chars
    returnCode = read_from_uart(data);

    // \n found, end of command
    if ( returnCode == OK ){ 
      printf("OK. Complete command for shakti: %s\n",data);
      checksum = 0;
      for (int i = 0; i < 15; i++ ){
        command[command_start+i] = data[i];
        checksum += data[i];  // add 16 values
      }
      write_to_uart(data); // send data back
      command_start = 0; // ready for next command
    }

    // no \n found so partial message
    if ( returnCode == PART ){ 
      printf("PART: <%s>\n",data);
      checksum = 0;
      for (int i = 0; i < 15; i++ ){
        command[command_start+i] = data[i];
        checksum += data[i];  // add 16 values
      }
      printf("Checksum=0x%x", checksum);
      write_to_uart(data); // send data back
// need more ack&nack and framenumber
      command_start += 16; // ready for next part of command
    }

    // end of data reached without \n , discard command
    if ( returnCode == EOD ){ 
      //printf("EOD command for shakti: <%s>\n", data);
      command_start = 0; // ready for next command
    }

    if ( command_start == 0 ) {
    // if we are not receiving a command, then get data for downlink
      getDataFromMPU6050();
      getDataFromHMC5883();
      getDataFromBME280();
      getDataFromDS3231();
  
      sprintf(buf, "Msg num: %d\n", cnt);
      write_to_uart(buf);
      printf(buf);
      for (int i=0; i< 16; i++) { buf[i] = 0x00; }
      cnt++;
      delay_loop(3000, 3000);
    }
  }
}
