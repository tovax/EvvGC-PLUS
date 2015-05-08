/*
    EvvGC-PLUS - Copyright (C) 2013-2015 Sarunas Vaitekonis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "hal.h"

#include "fixvector3d.h"
#include "fixquat.h"

#define MPU6050_I2C_ADDR_A0_LOW     0x68
#define MPU6050_I2C_ADDR_A0_HIGH    0x69

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_PWR_MGMT_1          0x6B

/* Sensor scales */
//#define MPU6050_GYRO_SCALE          F16(1.0f / 131.0f) //  250 deg/s
//#define MPU6050_GYRO_SCALE          F16(1.0f /  65.5f) //  500 deg/s
#define MPU6050_GYRO_SCALE          F16(1.0f /  32.8f) // 1000 deg/s
//#define MPU6050_GYRO_SCALE          F16(1.0f /  16.4f) // 2000 deg/s

#define GRAV                        (9.81)

//#define MPU6050_ACCEL_SCALE         F16(GRAV / 16384.0f) //  2G
//#define MPU6050_ACCEL_SCALE         F16(GRAV /  8192.0f) //  4G
#define MPU6050_ACCEL_SCALE         F16(GRAV /  4096.0f) //  8G
//#define MPU6050_ACCEL_SCALE         F16(GRAV /  2048.0f) // 16G

#define IMU_AXIS_ID_X               0x00
#define IMU_AXIS_ID_Y               0x01
#define IMU_AXIS_ID_Z               0x02

#define IMU_AXIS_DIR_POS            0x08
#define IMU_AXIS_ID_MASK            0x07

#define IMU1_AXIS_DIR_POS           0x08
#define IMU1_AXIS_ID_MASK           0x07
#define IMU1_CONF_MASK              0x0F

#define IMU2_AXIS_DIR_POS           0x80
#define IMU2_AXIS_ID_MASK           0x70
#define IMU2_CONF_MASK              0xF0

typedef struct tagIMUStruct {
  v3d accelData;        /* Accelerometer data.             */
  v3d gyroData;         /* Gyroscope data.                 */
  v3d accelBias;        /* Accelerometer bias.             */
  v3d gyroBias;         /* Gyroscope bias.                 */
  v3d accelFiltered;    /* Filtered accelerometer data.    */
  v3d v2Filtered;       /* Filtered direction of gravity.  */
  v3d accelError;       /* Gravity mismatch vector.        */
  v3d rpyIMU;           /* Attitude in Euler angles.       */
  qf16 qIMU;            /* Attitude quaternion of the IMU. */
  uint32_t clbrCounter; /* Calibration counter             */
  uint8_t axes_conf[3]; /* Configuration of IMU axes.      */
  uint8_t addr;         /* I2C address of the chip.        */
} __attribute__((packed)) IMUStruct, *PIMUStruct;

typedef struct tagI2CErrorStruct {
  i2cflags_t last_i2c_error;
  uint32_t i2c_error_counter;
  uint32_t i2c_timeout_counter;
} __attribute__((packed)) I2CErrorStruct, *PI2CErrorStruct;

/* IMU data structure. */
extern IMUStruct g_IMU1;
#if !defined(USE_ONE_IMU)
extern IMUStruct g_IMU2;
#endif /* USE_ONE_IMU */
/* Packed sensor settings. */
extern uint8_t g_sensorSettings[3];
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;

#ifdef __cplusplus
extern "C" {
#endif
  void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh);
  void imuCalibrationSet(uint8_t flags);
  uint8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAccel);
  uint8_t mpu6050Init(uint8_t addr);
  uint8_t mpu6050GetNewData(PIMUStruct pIMU);
  void accelBiasUpdate(PIMUStruct pIMU, const v3d *pNewSettings);
  void gyroBiasUpdate(PIMUStruct pIMU, const v3d *pNewSettings);
  void sensorSettingsUpdate(const uint8_t *pNewSettings);
#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
