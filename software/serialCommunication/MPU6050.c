#include"gpio_i2c.h"
#include "common.h"

#define I2C_WRITE 2
#define I2C_READ 3

#define MPU6050_SLAVE_ADDRESS      0xD0
#define MPU_6050_OUTPUT_BEGIN      0x3B
#define MPU6050_DLPF_CFG           0x20
#define MPU6050_SMPL_DIV           0x19
#define MPU6050_ADDRESS            0x68 
#define DEV_IDENTITY               0X68
#define DEV_IDENTITY_DATA_REG      0X75
#define MPU6050_REG_PWR_MGMT       0X6B
#define MPU6050_RESET              0x80
#define MPU6050_ACC_SENS_8G        0x10
#define MPU6050_REG_GYRO_CONFIG    0x1B // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG   0x1C // Accelerometer Configuration
#define MPU6050_GYRO_SENS_500D     0x08

#define PI 3.141592654

char readbuf[20];
int AccX,AccY,AccZ;
int accAngleX, accAngleY, GyroAngleX, GyroAngleY, GyroAngleZ;
int roll, pitch, yaw;
int AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int AccErrorX,AccErrory;
int GyroX, GyroY, GyroZ;
int c = 0;
int i=0;

//   soft float library need to be added, if hardfloat not there
 
void mpu6050_measuring_value(char *readbuf) {
  // read accelerometer data
  I2cSendSlaveAddress(MPU6050_SLAVE_ADDRESS, I2C_WRITE, 100);//selecting slave to be read
  I2cWriteData(MPU_6050_OUTPUT_BEGIN, 100);//selecting register to be read
  I2cSendSlaveAddress(MPU6050_SLAVE_ADDRESS, I2C_READ, 100);
  I2c_shakti_readbytes(readbuf,14,1, 100);//to read the output values
}

int mpu6050_init() {
  I2cInit();//configuring data and clocks as output lines

  printf("MPU6050 i2c Init completed\n");

  if (DEV_IDENTITY == I2c_Read_byte(MPU6050_SLAVE_ADDRESS, DEV_IDENTITY_DATA_REG, 100)){
    printf("MPU6050 SUCCESFULLY VERIFIED\n");
  } else{
    printf("something went wrong , device identity not verified\n");
    return -1;
  }
  //configuring power management register
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_REG_PWR_MGMT, MPU6050_RESET, 100);

  delay_loop(1000,1000);   // we can use waitfor() function also (delay time to reset the device and initialisations proper)

  //setting internal clock of MPU6050
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_REG_PWR_MGMT, 0x00, 100);

  // configuring accelerometer -set sensitivity scale factor +-8g
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACC_SENS_8G, 100);

  // configuring gyroscope-set  sensitivity scale factor to +-500deg/sec
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_SENS_500D, 100);

  // configuirng gyroscope output rate as 1khz
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_DLPF_CFG,0x01, 100);

  // configuring sample divider to 99(i.e sample rate= gyroscope output rate /(1+sample divider))
  I2c_Write_byte(MPU6050_SLAVE_ADDRESS, MPU6050_SMPL_DIV, 0x63, 100);//set sample rate to 10

  return 0;
}

void calculate_imu_error()
{
  int c = 0;
  int yaw;
  float accAngleX, accAngleY, GyroAngleX, GyroAngleY, GyroAngleZ;

  while (c < 200){
    mpu6050_measuring_value(&readbuf);

    AccErrorX = AccErrorX + (atan(AccY / sqrt(pow(AccX, 2)) + pow(AccZ, 2))) * 180 / PI;
    AccErrorY = AccErrorY + (atan(AccY / sqrt(pow(AccX, 2)) + pow(AccZ, 2))) * 180 / PI;

    GyroErrorX = GyroErrorX + (GyroX / 65.5);
    GyroErrorY = GyroErrorY + (GyroY / 65.5);
    GyroErrorZ = GyroErrorZ + (GyroZ / 65.5);
    c++;
  }
  c=0;
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrory
  // sum all reading
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  GyroX = GyroX - GyroErrorX ;     // GyroErrorX
  GyroY = GyroY - GyroErrorY;      // GyroErrorY
  GyroZ = GyroZ - GyroErrorZ;      // GyroErrorZ
}
