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

#ifndef _EVVGCP_H_
#define _EVVGCP_H_

#include "ch.h"
#include "hal.h"

#include "fixvector3d.h"
#include "fixquat.h"

/**
 * Global constants
 */
/* Earth gravity. */
#define GRAV                    9.81f

#define IMU1_CALIBRATE_ACC      0x00000008
#define IMU1_CALIBRATE_GYRO     0x00000010
#define IMU2_CALIBRATE_ACC      0x00000020
#define IMU2_CALIBRATE_GYRO     0x00000040
#define IMU1_CALIBRATE_MASK     0x00000018
#define IMU2_CALIBRATE_MASK     0x00000060
#define IMU_CALIBRATION_MASK    0x00000078

#define JUMP_TO_ROM_BOOTLOADER  0x01

/**
 * Global macros
 */
#define SYMVAL(sym) (uint32_t)(((uint8_t *)&(sym)) - ((uint8_t *)0))

/* I2C error info structure. */
typedef struct tagI2CErrorStruct {
  i2cflags_t last_i2c_error;
  uint32_t i2c_error_counter;
  uint32_t i2c_timeout_counter;
} __attribute__((packed)) I2CErrorStruct, *PI2CErrorStruct;

/**
 * Global variables
 */
/* The end of RAM. */
extern uint32_t __ram_end__;
/* Status of the board. */
extern uint32_t g_boardStatus;
/* Main thread termination flag. */
extern bool_t g_runMain;
/* I2C error info structure. */
extern I2CErrorStruct g_i2cErrorInfo;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _EVVGCP_H_ */