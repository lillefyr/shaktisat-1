#ifndef BME280_H
#define BME280_H

int read_bmp280_register(i2c_struct *i2c_instance, unsigned int reg_offset, unsigned int *readTemp, unsigned long delay);
int read_bmp280_values(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned long *pressure, unsigned long *temperature, unsigned long delay);
short read_bmp280_values16(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned long delay);
int write_bmp280_register(i2c_struct * i2c_instance, unsigned int reg_offset, unsigned char write_value, unsigned long delay);
int init_bmp280 ();

#endif
