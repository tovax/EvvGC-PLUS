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

/**
 * Glbal variables
 */
/* IMU data structure. */
extern IMUStruct g_IMU1;
#if !defined(USE_ONE_IMU)
extern IMUStruct g_IMU2;
#endif /* USE_ONE_IMU */
/* Packed sensor settings. */
extern uint8_t g_sensorSettings[3];

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
