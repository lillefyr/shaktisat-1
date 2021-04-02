#include "uart.h"
#include "pinmux.h"

#define SoftSerial uart_instance[1]

void main() {
  unsigned int baudrate = 4800; // good 38400, fail 19200, fail 9600

  *pinmux_config_reg = 0x05;

  printf("Set baudrate to %d\n", baudrate);
  set_baud_rate(SoftSerial, baudrate);

  delay_loop(1000, 1000);

  printf("Write one character to uart\n");
  write_uart_character(SoftSerial, 'A');

  printf("Working at %d baud\n", baudrate);
  while(1){
    delay_loop(1000, 1000);
  }
}

