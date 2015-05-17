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

/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "evvgcp.h"
#include "mpu6050.h"

/* C libraries: */
#include <string.h>

/* MPU6050 available addresses. */
#define MPU6050_I2C_ADDR_A0_LOW   0x68
#define MPU6050_I2C_ADDR_A0_HIGH  0x69

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48
#define MPU6050_PWR_MGMT_1        0x6B

/* MPU6050 gyroscopic sensor scales in fix16 format. */
//#define MPU6050_GYRO_SCALE        F16(1.0f / 131.0f) //  250 deg/s
//#define MPU6050_GYRO_SCALE        F16(1.0f /  65.5f) //  500 deg/s
#define MPU6050_GYRO_SCALE        F16(1.0f /  32.8f) // 1000 deg/s
//#define MPU6050_GYRO_SCALE        F16(1.0f /  16.4f) // 2000 deg/s

/* MPU6050 accelerometer scales in fix16 format. */
//#define MPU6050_ACCEL_SCALE       F16(GRAV / 16384.0f) //  2G
//#define MPU6050_ACCEL_SCALE       F16(GRAV /  8192.0f) //  4G
#define MPU6050_ACCEL_SCALE       F16(GRAV /  4096.0f) //  8G
//#define MPU6050_ACCEL_SCALE       F16(GRAV /  2048.0f) // 16G

#define IMU_AXIS_ID_X             0x00
#define IMU_AXIS_ID_Y             0x01
#define IMU_AXIS_ID_Z             0x02

#define IMU_AXIS_DIR_POS          0x08
#define IMU_AXIS_ID_MASK          0x07

#define IMU1_AXIS_DIR_POS         0x08
#define IMU1_AXIS_ID_MASK         0x07
#define IMU1_CONF_MASK            0x0F

#define IMU2_AXIS_DIR_POS         0x80
#define IMU2_AXIS_ID_MASK         0x70
#define IMU2_CONF_MASK            0xF0

/* MPU6050 IO buffer size in bytes. */
#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x05

/* I2C read transaction time-out in milliseconds.  */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01
/* Upper limit of calibration counter.             */
#define CALIBRATION_COUNTER_MAX   5000

/**
 * Global variables
 */
/* Default packed sensor settings.
 * Structure of the sensor settings is:
 * D2|2I2|1I2|0I2|D1|2I1|1I1|0I1
 * where Dx  - axis direction of the sensor x;
 *       nIx - n-th bit of axis ID of the sensor x.
 */
uint8_t g_sensorSettings[3] = {
  0x00 |
  IMU1_AXIS_DIR_POS |
  IMU2_AXIS_DIR_POS, /* Pitch (X) */
  0x11,              /* Roll  (Y) */
  0x22,              /* Yaw   (Z) */
};

/* IMU data structure. */
IMUStruct g_IMU1;
#if !defined(USE_ONE_IMU)
IMUStruct g_IMU2;
#endif /* USE_ONE_IMU */

/**
 * Local variables
 */
/* Data buffers */
static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrHigh - IMU address pin A0 is pulled high flag.
 */
void imuStructureInit(PIMUStruct pIMU, uint8_t fAddrHigh) {
  uint8_t i;
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  /* Initialize attitude quaternion to unity quaternion. */
  pIMU->qIMU.a = fix16_one;

  if (fAddrHigh) {
    pIMU->addr = MPU6050_I2C_ADDR_A0_HIGH;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] >> 4;
    }
  } else {
    pIMU->addr = MPU6050_I2C_ADDR_A0_LOW;
    for (i = 0; i < 3; i++) {
      pIMU->axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
    }
  }
}

/**
 * @brief
 */
void imuCalibrationSet(uint8_t flags) {
  g_boardStatus |= flags & IMU_CALIBRATION_MASK;
}

/**
 * @brief  Calibration function of IMU.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fCalibrateAccel - accelerometer calibration flag;
 * @return 0 - if calibration is not finished;
 *         1 - if calibration is finished.
 */
uint8_t imuCalibrate(PIMUStruct pIMU, uint8_t fCalibrateAccel) {
  if (fCalibrateAccel) {
    if (pIMU->clbrCounter == 0) {
      /* Reset accelerometer bias. */
      pIMU->accelData.z = fix16_add(pIMU->accelData.z, fix16_from_float(GRAV));
      memcpy((void *)&pIMU->accelBias, (void *)&pIMU->accelData, sizeof(pIMU->accelData));
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate accelerometer bias. */
      pIMU->accelData.z = fix16_add(pIMU->accelData.z, fix16_from_float(GRAV));
      v3d_add(&pIMU->accelBias, &pIMU->accelBias, &pIMU->accelData);
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update accelerometer bias. */
      v3d_div_s(&pIMU->accelBias, &pIMU->accelBias, fix16_from_int(CALIBRATION_COUNTER_MAX));
      pIMU->clbrCounter = 0;
    }
  } else {
    if (pIMU->clbrCounter == 0) {
      /* Reset gyroscope bias. */
      memcpy((void *)&pIMU->gyroBias, (void *)&pIMU->gyroData, sizeof(pIMU->gyroData));
      pIMU->clbrCounter++;
      return 0;
    } else if (pIMU->clbrCounter < CALIBRATION_COUNTER_MAX) {
      /* Accumulate gyroscope bias. */
      v3d_add(&pIMU->gyroBias, &pIMU->gyroBias, &pIMU->gyroData);
      pIMU->clbrCounter++;
      return 0;
    } else {
      /* Update gyroscope bias. */
      v3d_div_s(&pIMU->gyroBias, &pIMU->gyroBias, fix16_from_int(CALIBRATION_COUNTER_MAX));
      pIMU->clbrCounter = 0;
    }
  }
  return 1;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(uint8_t addr) {
  msg_t status = RDY_OK;
  i2cflags_t i2c_error = I2CD_NO_ERROR;

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b11000000;          // Register value;

  i2cAcquireBus(&I2CD2);

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    i2c_error = i2cGetErrors(&I2CD2);
    if (i2c_error != I2CD_NO_ERROR) {
      g_i2cErrorInfo.last_i2c_error = i2c_error;
      g_i2cErrorInfo.i2c_error_counter++;
    } else {
      g_i2cErrorInfo.i2c_timeout_counter++;
    }
    return 0;
  }

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0b00000011;          // Register value CLKSEL = PLL_Z;

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 2,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  if (status != RDY_OK) {
    i2cReleaseBus(&I2CD2);
    i2c_error = i2cGetErrors(&I2CD2);
    if (i2c_error != I2CD_NO_ERROR) {
      g_i2cErrorInfo.last_i2c_error = i2c_error;
      g_i2cErrorInfo.i2c_error_counter++;
    } else {
      g_i2cErrorInfo.i2c_timeout_counter++;
    }
    return 0;
  }

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 11;                  // SMPLRT_DIV register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0b00000000;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = 0b00010000;          // GYRO_CONFIG register value FS_SEL = +-1000 deg/s;
  mpu6050TXData[4] = 0b00010000;          // ACCEL_CONFIG register value AFS_SEL = +-8G;

  status = i2cMasterTransmitTimeout(&I2CD2, addr, mpu6050TXData, 5,
    NULL, 0, MS2ST(MPU6050_WRITE_TIMEOUT_MS));

  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    i2c_error = i2cGetErrors(&I2CD2);
    if (i2c_error != I2CD_NO_ERROR) {
      g_i2cErrorInfo.last_i2c_error = i2c_error;
      g_i2cErrorInfo.i2c_error_counter++;
    } else {
      g_i2cErrorInfo.i2c_timeout_counter++;
    }
    return 0;
  }

  return 1;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t mpu6050GetNewData(PIMUStruct pIMU) {
  msg_t status = RDY_OK;
  i2cflags_t i2c_error = I2CD_NO_ERROR;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, pIMU->addr, mpu6050TXData, 1,
    mpu6050RXData, 14, MS2ST(MPU6050_READ_TIMEOUT_MS));
  i2cReleaseBus(&I2CD2);

  if (status != RDY_OK) {
    i2c_error = i2cGetErrors(&I2CD2);
    if (i2c_error != I2CD_NO_ERROR) {
      g_i2cErrorInfo.last_i2c_error = i2c_error;
      g_i2cErrorInfo.i2c_error_counter++;
    } else {
      g_i2cErrorInfo.i2c_timeout_counter++;
    }
    return 0;
  }

  fix16_t *pAccelData = (fix16_t *)&pIMU->accelData;
  fix16_t *pGyroData  = (fix16_t *)&pIMU->gyroData;

  uint8_t id = pIMU->axes_conf[0] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[0] & IMU_AXIS_DIR_POS) {
    pAccelData[id] = fix16_mul(fix16_from_int((mpu6050RXData[0]<<8) | mpu6050RXData[1]), MPU6050_ACCEL_SCALE); /* Accel X */
    pGyroData[id]  = fix16_mul(fix16_from_int((mpu6050RXData[8]<<8) | mpu6050RXData[9]), MPU6050_GYRO_SCALE);  /* Gyro X  */
  } else {
    pAccelData[id] = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[0]<<8) | mpu6050RXData[1])), MPU6050_ACCEL_SCALE); /* Accel X */
    pGyroData[id]  = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[8]<<8) | mpu6050RXData[9])), MPU6050_GYRO_SCALE);  /* Gyro X  */
  }

  id = pIMU->axes_conf[1] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[1] & IMU_AXIS_DIR_POS) {
    pAccelData[id] = fix16_mul(fix16_from_int((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]), MPU6050_ACCEL_SCALE); /* Accel Y */
    pGyroData[id]  = fix16_mul(fix16_from_int((mpu6050RXData[10]<<8) | mpu6050RXData[11]), MPU6050_GYRO_SCALE);  /* Gyro Y  */
  } else {
    pAccelData[id] = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3])), MPU6050_ACCEL_SCALE); /* Accel Y */
    pGyroData[id]  = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[10]<<8) | mpu6050RXData[11])), MPU6050_GYRO_SCALE);  /* Gyro Y  */
  }

  id = pIMU->axes_conf[2] & IMU_AXIS_ID_MASK;
  if (pIMU->axes_conf[2] & IMU_AXIS_DIR_POS) {
    pAccelData[id] = fix16_mul(fix16_from_int((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]), MPU6050_ACCEL_SCALE); /* Accel Z */
    pGyroData[id]  = fix16_mul(fix16_from_int((mpu6050RXData[12]<<8) | mpu6050RXData[13]), MPU6050_GYRO_SCALE);  /* Gyro Z  */
  } else {
    pAccelData[id] = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5])), MPU6050_ACCEL_SCALE); /* Accel Z */
    pGyroData[id]  = fix16_mul(fix16_from_int(-1 - ((mpu6050RXData[12]<<8) | mpu6050RXData[13])), MPU6050_GYRO_SCALE);  /* Gyro Z  */
  }

  return 1;
}

/**
 * @brief
 */
void sensorSettingsUpdate(const uint8_t *pNewSettings) {
  uint8_t i;
  memcpy((void *)g_sensorSettings, (void *)pNewSettings, sizeof(g_sensorSettings));
  for (i = 0; i < 3; i++) {
    g_IMU1.axes_conf[i] = g_sensorSettings[i] & IMU1_CONF_MASK;
#if !defined(USE_ONE_IMU)
    g_IMU2.axes_conf[i] = g_sensorSettings[i] >> 4;
#endif /* USE_ONE_IMU */
  }
}

/**
 * @brief
 */
void accelBiasUpdate(PIMUStruct pIMU, const v3d *pNewSettings) {
  memcpy((void *)&pIMU->accelBias, (void *)pNewSettings, sizeof(pIMU->accelBias));
}

/**
 * @brief
 */
void gyroBiasUpdate(PIMUStruct pIMU, const v3d *pNewSettings) {
	memcpy((void *)&pIMU->gyroBias, (void *)pNewSettings, sizeof(pIMU->gyroBias));
}
