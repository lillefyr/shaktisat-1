#include"gpio_i2c.h"
#include "common.h"

#define I2C_WRITE 0
#define I2C_READ 1

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
int c = 0;
int i=0;

//   soft float library need to be added, if hardfloat not there
 
void mpu6050_measuring_value(char *readbuf) {
  // read accelerometer data
  I2cSendSlaveAddress(MPU6050_SLAVE_ADDRESS, I2C_WRITE, 100);//selecting slave to be read
  I2cWriteData(MPU_6050_OUTPUT_BEGIN, 100);//selecting register to be read
  I2cSendSlaveAddress(MPU6050_SLAVE_ADDRESS, I2C_READ, 100);
  I2c_shakti_readbytes(readbuf,14,1, 100);//to read the output values

/*
  printf("AccX: %x\n", (readbuf[0]<<8 |readbuf[1])  );
  printf("AccY: %x\n", (readbuf[2]<<8 |readbuf[3])  );
  printf("AccZ: %x\n", (readbuf[4]<<8 |readbuf[5])  );

  printf("GyroX: %x\n", (readbuf[8]<<8 |readbuf[9]) );
  printf("GyroY: %x\n", (readbuf[10]<<8 |readbuf[11]) );
  printf("GyroZ: %x\n", (readbuf[12]<<8 |readbuf[13]) );
*/
}

int mpu6050_init() {
  I2cInit();//configuring data and clocks as output lines

  printf("\nMPU6050 i2c Init completed\n");

  int kurt = I2c_Read_byte(MPU6050_SLAVE_ADDRESS, DEV_IDENTITY_DATA_REG, 100);
  if (DEV_IDENTITY == kurt) { //I2c_Read_byte(MPU6050_SLAVE_ADDRESS, DEV_IDENTITY_DATA_REG, 100)){
    printf("MPU6050 SUCCESFULLY VERIFIED\n");
  } else{
    printf("ERROR: MPU6050 identity not verified found ID=0x%x expected 0x%x\n",kurt, DEV_IDENTITY);
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

void calculate_imu_error(int *GyroX, int *GyroY, int *GyroZ, int *accAngleX, int *accAngleY) {
// this is only to calibrateh the MPU6050 and should not be in flight software
  int c = 0;
#define C_MAX 200
//  float accAngleX, accAngleY, GyroAngleX, GyroAngleY, GyroAngleZ;
//  float AccErrorX, AccErrorY;
//  float GyroErrorX, GyroErrorY, GyroErrorZ;
  int AccErrorX, AccErrorY;
  int GyroErrorX, GyroErrorY, GyroErrorZ;
  int AccX,AccY,AccZ;
  int GyroAngleX, GyroAngleY, GyroAngleZ;

  while (c < C_MAX){
    printf("Counter = %d\n", c);
    mpu6050_measuring_value(&readbuf);

// divide by 4096 may be wrong
    AccX = (readbuf[0]<<8 |readbuf[1]) / 4096; //16-bit X-axis data
    AccY = (readbuf[2]<<8 |readbuf[3]) / 4096; //16-bit Y-axis data
    AccZ = (readbuf[4]<<8 |readbuf[5]) / 4096; //16-bit Z-axis data

    *GyroX = (readbuf[8]<<8 |readbuf[9]);
    *GyroY = (readbuf[10]<<8 |readbuf[11]);
    *GyroZ = (readbuf[12]<<8 |readbuf[13]);

    AccErrorX = AccErrorX + (atan(AccY / sqrt(pow(AccX, 2)) + pow(AccZ, 2))) * 180 / PI;
    AccErrorY = AccErrorY + (atan(AccY / sqrt(pow(AccX, 2)) + pow(AccZ, 2))) * 180 / PI;

    GyroErrorX = GyroErrorX + (*GyroX / 65.5);
    GyroErrorY = GyroErrorY + (*GyroY / 65.5);
    GyroErrorZ = GyroErrorZ + (*GyroZ / 65.5);
    c++;
  }

  AccErrorX = AccErrorX / C_MAX;
  AccErrorY = AccErrorY / C_MAX;

  // return the calculated values
  *accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX
  *accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY

  // sum all reading
  GyroErrorX = GyroErrorX / C_MAX;
  GyroErrorY = GyroErrorY / C_MAX;
  GyroErrorZ = GyroErrorZ / C_MAX;

  //  return the calculated values
  *GyroX = *GyroX - GyroErrorX;     // GyroErrorX
  *GyroY = *GyroY - GyroErrorY;      // GyroErrorY
  *GyroZ = *GyroZ - GyroErrorZ;      // GyroErrorZ
}
