#ifndef BME280_SOFTI2C.H
#define BME280_SOFTI2C.H

int read_bmp280_soft_I2C_register(unsigned int reg_offset, unsigned int *readTemp, unsigned long delay);
int read_bmp280_soft_I2C_values(unsigned int reg_offset, unsigned long *pressure, unsigned long *temperature, unsigned long delay);
short read_bmp280_soft_I2C_values16(unsigned int reg_offset, unsigned long delay);
int write_bmp280_soft_I2C_register(unsigned int reg_offset, unsigned char write_value, unsigned long delay);
int bmp280_soft_I2C_init ();

#endif
