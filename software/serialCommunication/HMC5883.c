#include <stdint.h>//Includes the definitions of standard input/output functions//
#include "i2c.h"
#include "log.h"
#include "uart.h"
#include "common.h"
#include "HMC5883.h"

#define PI 3.141592654


int hmc5883_init() {
  printf("hmc5883_init\n");

  i2c_init();

  if(config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT)) {
    log_error("Something Wrong In Initialization\n");
    return -1;
  }

  declination_offset_radians = 0;
  mode = COMPASS_SINGLE | COMPASS_SCALE_130 | COMPASS_HORIZONTAL_X_NORTH;  
}

/** Set declination in degrees, minutes and direction (E/W)
 *   See http://www.magnetic-declination.com/
 */

void hmc5883_SetDeclination( int declination_degs , int declination_mins, char declination_dir ) {    
  // Convert declination to decimal degrees
  switch(declination_dir)
  {
    // North and East are positive   
    case 'E': 
      declination_offset_radians = ( declination_degs + (1/60 * declination_mins)) * (PI / 180);
      break;
      
    // South and West are negative    
    case 'W':
      declination_offset_radians =  0 - (( declination_degs + (1/60 * declination_mins) ) * (PI / 180));
      break;
  } 
}

/** Set the sampling mode to one of COMPASS_CONTINUOUS or COMPASS_SINGLE
 */

void hmc5883_SetSamplingMode( i2c_struct * i2c_instance, uint16_t sampling_mode, unsigned long int delay) {  
  // Mode is the bits marked M in mode
  //    xxxxxxxxxxxSSSMM
  mode = (mode & ~0x03) | (sampling_mode & 0x03);

  hmc5883_Write(i2c_instance, COMPASS_MODE_REGISTER, mode & 0x03, delay);
}

/** Set the scale to one of COMPASS_SCALE_088 through COMPASS_SCALE_810
 * Higher scales are less sensitive and less noisy
 * Lower scales are more sensitive and more noisy
 */

void hmc5883_SetScale( i2c_struct * i2c_instance, uint16_t scale, unsigned long int delay ) {
  // Scale is the bits marked S in mode
  //    xxxxxxxxxxxSSSMM  
  mode = (mode & ~0x1C) | (scale & 0x1C);

  hmc5883_Write(i2c_instance, COMPASS_CONFIG_REGISTER_B, (( mode >> 2 ) & 0x07) << 5, delay);
}

/** Set the orientation to one of COMPASS_HORIZONTAL_X_NORTH 
 * through COMPASS_VERTICAL_Y_WEST
 *  
 */

void hmc5883_SetOrientation( uint16_t orientation ) {
  // Orientation is the bits marked XXXYYYZZZ in mode
  //    xxXXXYYYZZZxxxxx
  mode = (mode & ~0x3FE0) | (orientation & 0x3FE0);    
}

/** Get the heading of the compass in degrees. */
float hmc5883_GetHeadingDegrees(i2c_struct * i2c_instance, unsigned long delay) {     
  // Obtain a sample of the magnetic axes
  struct MagnetometerSample sample = hmc5883_ReadAxes(i2c_instance, delay);
  
  float heading;    
  
  // Determine which of the Axes to use for North and West (when compass is "pointing" north)
  float mag_north, mag_west;
   
  // Z = bits 0-2
  switch((mode >> 5) & 0x07 )
  {
    case COMPASS_NORTH: mag_north = sample.Z; break;
    case COMPASS_SOUTH: mag_north = 0-sample.Z; break;
    case COMPASS_WEST:  mag_west  = sample.Z; break;
    case COMPASS_EAST:  mag_west  = 0-sample.Z; break;
      
    // Don't care
    case COMPASS_UP:
    case COMPASS_DOWN:
    break;
  }
  
  // Y = bits 3 - 5
  switch(((mode >> 5) >> 3) & 0x07 )
  {
    case COMPASS_NORTH: mag_north = sample.Y;  break;
    case COMPASS_SOUTH: mag_north = 0-sample.Y; ;  break;
    case COMPASS_WEST:  mag_west  = sample.Y;  break;
    case COMPASS_EAST:  mag_west  = 0-sample.Y;  break;
      
    // Don't care
    case COMPASS_UP:
    case COMPASS_DOWN:
    break;
  }
  
  // X = bits 6 - 8
  switch(((mode >> 5) >> 6) & 0x07 )
  {
    case COMPASS_NORTH: mag_north = sample.X; break;
    case COMPASS_SOUTH: mag_north = 0-sample.X; break;
    case COMPASS_WEST:  mag_west  = sample.X; break;
    case COMPASS_EAST:  mag_west  = 0-sample.X; break;
      
    // Don't care
    case COMPASS_UP:
    case COMPASS_DOWN:
    break;
  }
    
  // calculate heading from the north and west magnetic axes
  heading = atan2(mag_west, mag_north);
  
  // Adjust the heading by the declination
  heading += declination_offset_radians;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  return heading * 180/PI; 
}


/** Read the axes from the magnetometer.
 * In SINGLE mode we take a sample.  In CONTINUOUS mode we 
 * just grab the most recent result in the registers.
 */

struct MagnetometerSample hmc5883_ReadAxes(i2c_struct * i2c_instance, unsigned long delay) {
  if(mode & COMPASS_SINGLE) {    
    hmc5883_Write(i2c_instance, COMPASS_MODE_REGISTER, (uint8_t)( mode & 0x03 ), delay);  
    //delay(66); // We could listen to the data ready pin instead of waiting.
  }

  uint8_t buffer[6] ;
  hmc5883_Read(i2c_instance, buffer, 6, delay);

  struct MagnetometerSample sample;
  
  // NOTE:
  // The registers are in the order X Z Y  (page 11 of datasheet)
  // the datasheet when it describes the registers details then in order X Y Z (page 15)
  // stupid datasheet writers
  sample.X = (buffer[0] << 8) | buffer[1];  
  sample.Z = (buffer[2] << 8) | buffer[3];
  sample.Y = (buffer[4] << 8) | buffer[5];
  
  return sample;
}

/** Write data to the compass by I2C */
uint8_t hmc5883_Write(i2c_struct * i2c_instance, uint8_t address, uint8_t data, unsigned long delay) {
  //Writes the slave address for write
  uint8_t ret = i2c_send_slave_address(i2c_instance, address, I2C_WRITE, 800);
  if ( ret != 0 ) {
    printf("hmc5883: send to slave address failed 0x%x return = %d\n", COMPASS_I2C_ADDRESS, ret);
    return ret;
  }

  ret = i2c_write_data(i2c_instance, &data, delay);
  if ( ret != 0 ) { return ret; }

  //Stops the I2C transaction
  i2c_instance->control = I2C_STOP;
  
  return 0;
}

/** Read data from the compass by I2C  
 */
uint8_t hmc5883_Read(i2c_struct * i2c_instance, uint8_t buffer[], uint8_t length, unsigned long delay) {
  // Write the register address that we will begin the read from, this
  // has the effect of "seeking" to that register

  unsigned char temp = 0;
  uint8_t ret;

  //Writes the slave address for read
  ret = i2c_send_slave_address(i2c_instance, COMPASS_I2C_ADDRESS, I2C_WRITE, 800);
  if ( ret != 0 ) {
    printf("hmc5883: send to slave address failed 0x%x return = %d\n", COMPASS_I2C_ADDRESS, ret);
    return ret;
  }

  //Writes the pointer to address that needs to be read
  ret = i2c_write_data(i2c_instance, COMPASS_I2C_ADDRESS, delay);
  if ( ret != 0 ) { return ret; }

  //Stops the I2C transaction to start reading the temperature value.
  i2c_instance->control = I2C_STOP;
  
  ret = i2c_send_slave_address(i2c_instance, COMPASS_I2C_ADDRESS, I2C_READ, 800);
  if ( ret != 0 ) {
    printf("hmc5883: send to slave address failed 0x%x return = %d\n", COMPASS_I2C_ADDRESS, ret);
    return ret;
  }

  temp = 0;
  // Make a dummy read as per spec of the I2C controller
  ret = i2c_read_data(i2c_instance, &temp, delay);

  for(uint8_t i = 0; i < length; i++) {
    ret = i2c_read_data(i2c_instance, &temp, delay);
    buffer[i] = temp;
    if( i == (length - 1) ){
      i2c_instance->control = I2C_NACK;
    }
  }
  i2c_instance->control = I2C_STOP;

  return 0;
}

int getDataFromHMC5883( i2c_struct * i2c_instance, float *heading, unsigned long int delay){
  // Magnetic Declination is the correction applied according to your present location
  // in order to get True North from Magnetic North, it varies from place to place.
  //
  // The declination for your area can be obtained from http://www.magnetic-declination.com/
  // Take the "Magnetic Declination" line that it gives you in the information,
  //
  // Examples:
  //   Christchurch, 23° 35' EAST
  //   Wellington  , 22° 14' EAST
  //   Dunedin     , 25° 8'  EAST
  //   Auckland    , 19° 30' EAST
  //   Baden Argau , 2   55  East
  //
  hmc5883_SetDeclination(2, 55, 'E');

  // The device can operate in SINGLE (default) or CONTINUOUS mode
  //   SINGLE simply means that it takes a reading when you request one
  //   CONTINUOUS means that it is always taking readings
  // for most purposes, SINGLE is what you want.
  hmc5883_SetSamplingMode(i2c_instance, COMPASS_SINGLE, delay);

  // The scale can be adjusted to one of several levels, you can probably leave it at the default.
  // Essentially this controls how sensitive the device is.
  //   Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
  // Specify the option as COMPASS_SCALE_xxx
  // Lower values are more sensitive, higher values are less sensitive.
  // The default is probably just fine, it works for me.  If it seems very noisy
  // (jumping around), incrase the scale to a higher one.
  hmc5883_SetScale( i2c_instance, COMPASS_SCALE_130, delay );

  // The compass has 3 axes, but two of them must be close to parallel to the earth's surface to read it,
  // (we do not compensate for tilt, that's a complicated thing) - just like a real compass has a floating
  // needle you can imagine the digital compass does too.
  //
  // To allow you to mount the compass in different ways you can specify the orientation:
  //   COMPASS_HORIZONTAL_X_NORTH (default), the compass is oriented horizontally, top-side up. when pointing North the X silkscreen arrow will point North
  //   COMPASS_HORIZONTAL_Y_NORTH, top-side up, Y is the needle,when pointing North the Y silkscreen arrow will point North
  //   COMPASS_VERTICAL_X_EAST,    vertically mounted (tall) looking at the top side, when facing North the X silkscreen arrow will point East
  //   COMPASS_VERTICAL_Y_WEST,    vertically mounted (wide) looking at the top side, when facing North the Y silkscreen arrow will point West
  hmc5883_SetOrientation( COMPASS_HORIZONTAL_X_NORTH );

  *heading = hmc5883_GetHeadingDegrees( i2c_instance, delay);
  printf("%heading=%f\n", *heading);
  return 0;
}
