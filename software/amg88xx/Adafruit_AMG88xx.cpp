#include "Adafruit_AMG88xx.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
//#define I2C_DEBUG
//#include "i2c.h"
#include "log.h"
#include "uart.h"
#include "Wire.h"
#include <time.h>

#if defined(ESP32)
// https://github.com/espressif/arduino-esp32/issues/839
#define AMG_I2C_CHUNKSIZE 16
#else
#define AMG_I2C_CHUNKSIZE 32
#endif
/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is
   0x69
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/

inline int min(int a,int b) {return ((a)<(b)?(a):(b)); }
long constrain(long x, long a, long b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
  
    // Storing start time
    clock_t start_time = clock();
  
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

bool Adafruit_AMG88xx::begin(int addr) {
  _i2caddr = addr;

  _i2c_init();

  // enter normal mode
  _pctl.PCTL = AMG88xx_NORMAL_MODE;
  write8(AMG88xx_PCTL, _pctl.get());

  // software reset
  _rst.RST = AMG88xx_INITIAL_RESET;
  write8(AMG88xx_RST, _rst.get());

  // disable interrupts by default
  disableInterrupt();

  // set to 10 FPS
  _fpsc.FPS = AMG88xx_FPS_10;
  write8(AMG88xx_FPSC, _fpsc.get());

  delay(1000);
  return true;
}

/**************************************************************************/
/*!
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void Adafruit_AMG88xx::setMovingAverageMode(bool mode) {
  _ave.MAMOD = mode;
  write8(AMG88xx_AVE, _ave.get());
}

/**************************************************************************/
/*!
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 *
   high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptLevels(float high, float low) {
  setInterruptLevels(high, low, high * .95);
}

/**************************************************************************/
/*!
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptLevels(float high, float low,
                                          float hysteresis) {
  int highConv = high / AMG88xx_PIXEL_TEMP_CONVERSION;
  highConv = constrain(highConv, -4095, 4095);
  _inthl.INT_LVL_H = highConv & 0xFF;
  _inthh.INT_LVL_H = (highConv & 0xF) >> 4;
  this->write8(AMG88xx_INTHL, _inthl.get());
  this->write8(AMG88xx_INTHH, _inthh.get());

  int lowConv = low / AMG88xx_PIXEL_TEMP_CONVERSION;
  lowConv = constrain(lowConv, -4095, 4095);
  _intll.INT_LVL_L = lowConv & 0xFF;
  _intlh.INT_LVL_L = (lowConv & 0xF) >> 4;
  this->write8(AMG88xx_INTLL, _intll.get());
  this->write8(AMG88xx_INTLH, _intlh.get());

  int hysConv = hysteresis / AMG88xx_PIXEL_TEMP_CONVERSION;
  hysConv = constrain(hysConv, -4095, 4095);
  _ihysl.INT_HYS = hysConv & 0xFF;
  _ihysh.INT_HYS = (hysConv & 0xF) >> 4;
  this->write8(AMG88xx_IHYSL, _ihysl.get());
  this->write8(AMG88xx_IHYSH, _ihysh.get());
}

/**************************************************************************/
/*!
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void Adafruit_AMG88xx::enableInterrupt() {
  _intc.INTEN = 1;
  this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void Adafruit_AMG88xx::disableInterrupt() {
  _intc.INTEN = 0;
  this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode,
   AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void Adafruit_AMG88xx::setInterruptMode(int mode) {
  _intc.INTMOD = mode;
  this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*!
    @brief  Read the state of the triggered interrupts on the device. The full
   interrupt register is 8 ints in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of ints to read. Default is 8 ints.
    @returns up to 8 ints of data in buf
*/
/**************************************************************************/
void Adafruit_AMG88xx::getInterrupt(int *buf, int size) {
  int intsToRead = min(size, (int)8);

  this->read(AMG88xx_INT_OFFSET, buf, intsToRead);
}

/**************************************************************************/
/*!
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void Adafruit_AMG88xx::clearInterrupt() {
  _rst.RST = AMG88xx_FLAG_RESET;
  write8(AMG88xx_RST, _rst.get());
}

/**************************************************************************/
/*!
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_AMG88xx::readThermistor() {
  int raw[2];
  this->read(AMG88xx_TTHL, raw, 2);
  long recast = ((long)raw[1] << 8) | ((long)raw[0]);

  return signedMag12ToFloat(recast) * AMG88xx_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*!
    @brief  Read Infrared sensor values
    @param  buf the array to place the pixels in
    @param  size Optionsl number of ints to read (up to 64). Default is 64
   ints.
    @return up to 64 ints of pixel data in buf
*/
/**************************************************************************/
void Adafruit_AMG88xx::readPixels(float *buf, int size) {
  long recast;
  float converted;
  int intsToRead =
      min((int)(size << 1), (int)(AMG88xx_PIXEL_ARRAY_SIZE << 1));
  int rawArray[intsToRead];
  this->read(AMG88xx_PIXEL_OFFSET, rawArray, intsToRead);

  for (int i = 0; i < size; i++) {
    int pos = i << 1;
    recast = ((long)rawArray[pos + 1] << 8) | ((long)rawArray[pos]);

    converted = int12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
    buf[i] = converted;
  }
}

/**************************************************************************/
/*!
    @brief  write one int of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void Adafruit_AMG88xx::write8(int reg, int value) {
  this->write(reg, &value, 1);
}

/**************************************************************************/
/*!
    @brief  read one int of data from the specified register
    @param  reg the register to read
    @returns one int of register data
*/
/**************************************************************************/
int Adafruit_AMG88xx::read8(int reg) {
  int ret;
  this->read(reg, &ret, 1);

  return ret;
}

void Adafruit_AMG88xx::_i2c_init() { Wire.begin(); }

void Adafruit_AMG88xx::read(int reg, int *buf, int num) {
  int value;
  int pos = 0;

  // on arduino we need to read in AMG_I2C_CHUNKSIZE int chunks
  while (pos < num) {
    int read_now = min((int)AMG_I2C_CHUNKSIZE, (int)(num - pos));
    Wire.beginTransmission((int)_i2caddr);
    Wire.write((int)reg + pos);
    Wire.endTransmission();
    Wire.requestFrom((int)_i2caddr, read_now);

#ifdef I2C_DEBUG
    printf("[$");
    printf(reg + pos, HEX);
    printf("] -> ");
#endif
    for (int i = 0; i < read_now; i++) {
      buf[pos] = Wire.read();
#ifdef I2C_DEBUG
      printf("0x");
      printf(buf[pos], HEX);
      printf(", ");
#endif
      pos++;
    }
#ifdef I2C_DEBUG
    printfln();
#endif
  }
}

void Adafruit_AMG88xx::write(int reg, int *buf, int num) {
#ifdef I2C_DEBUG
  printf("[$");
  printf(reg, HEX);
  printf("] <- ");
#endif
  Wire.beginTransmission((int)_i2caddr);
  Wire.write((int)reg);
  for (int i = 0; i < num; i++) {
    Wire.write(buf[i]);
#ifdef I2C_DEBUG
    printf("0x");
    printf(buf[i], HEX);
    printf(", ");
#endif
  }
  Wire.endTransmission();
#ifdef I2C_DEBUG
  printfln();
#endif
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit signed magnitude value to a floating point number
    @param  val the 12-bit signed magnitude value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float Adafruit_AMG88xx::signedMag12ToFloat(long val) {
  // take first 11 bits as absolute val
  long absVal = (val & 0x7FF);

  return (val & 0x800) ? 0 - (float)absVal : (float)absVal;
}

/**************************************************************************/
/*!
    @brief  convert a 12-bit integer two's complement value to a floating point
   number
    @param  val the 12-bit integer  two's complement value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
float Adafruit_AMG88xx::int12ToFloat(long val) {
  long sVal = (val << 4); // shift to left so that sign bit of 12 bit integer number is
                  // placed on sign bit of 16 bit signed integer number
  return sVal >> 4; // shift back the signed number, return converts to float
}
