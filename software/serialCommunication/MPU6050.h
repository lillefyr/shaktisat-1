#ifndef MPU6050_H
#define MPU6050_H

void mpu6050_measuring_value(char *readbuf);
int mpu6050_init();
void calculate_imu_error(int *GyroX, int *GyroY, int *GyroZ, int *accAngleX, int *accAngleY);

#endif
