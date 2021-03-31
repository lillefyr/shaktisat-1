/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch tries to read the pixels from the sensor

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Wire.h"
#include "Adafruit_AMG88xx.h"
#include <stdio.h>

extern "C"
{
void *__dso_handle = NULL;
}

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

int main() {
  //Serial.begin(9600);
    printf("\nAMG88xx pixels");

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        printf("\nCould not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    printf("\n-- Pixels Test --");

    printf("\n");

    //delay(1000); // let sensor boot up
    while(1) {
      amg.readPixels(pixels);

      printf("\n[");
      for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
        printf("pixels %f", pixels[i-1]);
        printf(", ");
        if( i%8 == 0 ) printf("\n");
      }
      printf("\n]");
      printf("\n]");

      //delay a second
      //delay(1000);
    }
  return 0;
}

