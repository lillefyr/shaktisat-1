#ifndef LIB_ADAFRUIT_AMG88XX_H
#define LIB_ADAFRUIT_AMG88XX_H

//#include "Arduino.h"

#include "Wire.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define AMG88xx_ADDRESS (0x69)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {
  AMG88xx_PCTL = 0x00,
  AMG88xx_RST = 0x01,
  AMG88xx_FPSC = 0x02,
  AMG88xx_INTC = 0x03,
  AMG88xx_STAT = 0x04,
  AMG88xx_SCLR = 0x05,
  // 0x06 reserved
  AMG88xx_AVE = 0x07,
  AMG88xx_INTHL = 0x08,
  AMG88xx_INTHH = 0x09,
  AMG88xx_INTLL = 0x0A,
  AMG88xx_INTLH = 0x0B,
  AMG88xx_IHYSL = 0x0C,
  AMG88xx_IHYSH = 0x0D,
  AMG88xx_TTHL = 0x0E,
  AMG88xx_TTHH = 0x0F,
  AMG88xx_INT_OFFSET = 0x010,
  AMG88xx_PIXEL_OFFSET = 0x80
};

enum power_modes {
  AMG88xx_NORMAL_MODE = 0x00,
  AMG88xx_SLEEP_MODE = 0x01,
  AMG88xx_STAND_BY_60 = 0x20,
  AMG88xx_STAND_BY_10 = 0x21
};

enum sw_resets { AMG88xx_FLAG_RESET = 0x30, AMG88xx_INITIAL_RESET = 0x3F };

enum frame_rates { AMG88xx_FPS_10 = 0x00, AMG88xx_FPS_1 = 0x01 };

enum int_enables { AMG88xx_INT_DISABLED = 0x00, AMG88xx_INT_ENABLED = 0x01 };

enum int_modes { AMG88xx_DIFFERENCE = 0x00, AMG88xx_ABSOLUTE_VALUE = 0x01 };

/*=========================================================================*/

#define AMG88xx_PIXEL_ARRAY_SIZE 64
#define AMG88xx_PIXEL_TEMP_CONVERSION .25
#define AMG88xx_THERMISTOR_CONVERSION .0625

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with AMG88xx
   IR sensor chips
*/
/**************************************************************************/

class Adafruit_AMG88xx {
public:
  // constructors
  Adafruit_AMG88xx(void){};
  ~Adafruit_AMG88xx(void){};

  bool begin(int addr = AMG88xx_ADDRESS);

  void readPixels(float *buf, int size = AMG88xx_PIXEL_ARRAY_SIZE);
  float readThermistor();

  void setMovingAverageMode(bool mode);

  void enableInterrupt();
  void disableInterrupt();
  void setInterruptMode(int mode);
  void getInterrupt(int *buf, int size = 8);
  void clearInterrupt();

  // this will automatically set hysteresis to 95% of the high value
  void setInterruptLevels(float high, float low);

  // this will manually set hysteresis
  void setInterruptLevels(float high, float low, float hysteresis);

private:
  int _i2caddr;

  void write8(int reg, int value);
  void write16(int reg, long value);
  int read8(int reg);

  void read(int reg, int *buf, int num);
  void write(int reg, int *buf, int num);
  void _i2c_init();

  float signedMag12ToFloat(long val);
  float int12ToFloat(long val);

  // The power control register
  struct pctl {
    // 0x00 = Normal Mode
    // 0x01 = Sleep Mode
    // 0x20 = Stand-by mode (60 sec intermittence)
    // 0x21 = Stand-by mode (10 sec intermittence)

    int PCTL : 8;

    int get() { return PCTL; }
  };
  pctl _pctl;

  // reset register
  struct rst {
    // 0x30 = flag reset (all clear status reg 0x04, interrupt flag and
    // interrupt table) 0x3F = initial reset (brings flag reset and returns to
    // initial setting)

    int RST : 8;

    int get() { return RST; }
  };
  rst _rst;

  // frame rate register
  struct fpsc {

    // 0 = 10FPS
    // 1 = 1FPS
    int FPS : 1;

    int get() { return FPS & 0x01; }
  };
  fpsc _fpsc;

  // interrupt control register
  struct intc {

    // 0 = INT output reactive (Hi-Z)
    // 1 = INT output active
    int INTEN : 1;

    // 0 = Difference interrupt mode
    // 1 = absolute value interrupt mode
    int INTMOD : 1;

    int get() { return (INTMOD << 1 | INTEN) & 0x03; }
  };
  intc _intc;

  // status register
  struct stat {
    int unused : 1;
    // interrupt outbreak (val of interrupt table reg)
    int INTF : 1;

    // temperature output overflow (val of temperature reg)
    int OVF_IRS : 1;

    // thermistor temperature output overflow (value of thermistor)
    int OVF_THS : 1;

    int get() {
      return ((OVF_THS << 3) | (OVF_IRS << 2) | (INTF << 1)) & 0x0E;
    }
  };
  stat _stat;

  // status clear register
  // write to clear overflow flag and interrupt flag
  // after writing automatically turns to 0x00
  struct sclr {
    int unused : 1;
    // interrupt flag clear
    int INTCLR : 1;
    // temp output overflow flag clear
    int OVS_CLR : 1;
    // thermistor temp output overflow flag clear
    int OVT_CLR : 1;

    int get() {
      return ((OVT_CLR << 3) | (OVS_CLR << 2) | (INTCLR << 1)) & 0x0E;
    }
  };
  sclr _sclr;

  // average register
  // for setting moving average output mode
  struct ave {
    int unused : 5;
    // 1 = twice moving average mode
    int MAMOD : 1;

    int get() { return (MAMOD << 5); }
  };
  struct ave _ave;

  // interrupt level registers
  // for setting upper / lower limit hysteresis on interrupt level

  // interrupt level upper limit setting. Interrupt output
  // and interrupt pixel table are set when value exceeds set value
  struct inthl {
    int INT_LVL_H : 8;

    int get() { return INT_LVL_H; }
  };
  struct inthl _inthl;

  struct inthh {
    int INT_LVL_H : 4;

    int get() { return INT_LVL_H; }
  };
  struct inthh _inthh;

  // interrupt level lower limit. Interrupt output
  // and interrupt pixel table are set when value is lower than set value
  struct intll {
    int INT_LVL_L : 8;

    int get() { return INT_LVL_L; }
  };
  struct intll _intll;

  struct intlh {
    int INT_LVL_L : 4;

    int get() { return (INT_LVL_L & 0xF); }
  };
  struct intlh _intlh;

  // setting of interrupt hysteresis level when interrupt is generated.
  // should not be higher than interrupt level
  struct ihysl {
    int INT_HYS : 8;

    int get() { return INT_HYS; }
  };
  struct ihysl _ihysl;

  struct ihysh {
    int INT_HYS : 4;

    int get() { return (INT_HYS & 0xF); }
  };
  struct ihysh _ihysh;

  // thermistor register
  // SIGNED MAGNITUDE FORMAT
  struct tthl {
    int TEMP : 8;

    int get() { return TEMP; }
  };
  struct tthl _tthl;

  struct tthh {
    int TEMP : 3;
    int SIGN : 1;

    int get() { return ((SIGN << 3) | TEMP) & 0xF; }
  };
  struct tthh _tthh;

  // temperature registers 0x80 - 0xFF
  /*
  //read to indicate temperature data per 1 pixel
  //SIGNED MAGNITUDE FORMAT
  struct t01l {
          int TEMP : 8;

          int get(){
                  return TEMP;
          }
  };
  struct t01l _t01l;

  struct t01h {
          int TEMP : 3;
          int SIGN : 1;

          int get(){
                  return ( (SIGN << 3) | TEMP) & 0xF;
          }
  };
  struct t01h _t01h;
  */
};

#endif
