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

/*
  References:
  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary
      Filters on the Special Orthogonal Group", Automatic Control,
      IEEE Transactions on, vol.53, no.5, pp.1203,1218, June 2008
  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T.,
      "A complementary filter for attitude estimation of a fixed-wing UAV",
      Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International
      Conference on, vol., no., pp.340,345, 22-26 Sept. 2008
*/

/*
  Note:
  - the order of rotation of the standard 3D-gimbal system is:
    Pitch (X) then Roll (Y) and then Yaw (Z).
*/

/* C libraries: */
#include <string.h>

#include "misc.h"
#include "attitude.h"
#include "pwmio.h"

#define FIXED_DT_STEP             0.0015f

#define ACCEL_TAU                 0.1f
#define INPUT_SIGNAL_ALPHA        266.7f

#define MODE_FOLLOW_DEAD_BAND     F16(M_PI / 36.0f)

#define MOTOR_STEP_LIMIT_MAX      F16(M_PI / 45.0f)
#define MOTOR_STEP_LIMIT_MIN      F16(M_PI /-45.0f)

#define GRAV_LIMIT_LOW            F16(9.81f * 0.6f)
#define GRAV_LIMIT_HIGH           F16(9.81f * 1.4f)

/* PID controller structure. */
typedef struct tagPIDStruct {
  fix16_t P;
  fix16_t I;
  fix16_t D;
  fix16_t F;
  fix16_t prevDistance;
  fix16_t prevSpeed;
  fix16_t prevDisturbance;
  fix16_t prevCommand;
} __attribute__((packed)) PIDStruct, *PPIDStruct;

/**
 * Global variables.
 */
/* Mechanical offset of the motors. */
fix16_t g_motorOffset[3] = {0, 0, 0};

/**
 * Default closed loop with feed forward settings.
 */
PIDSettings g_pidSettings[3] = {
/* P, I, D, F */
  {0, 0, 0, 0}, /* Pitch PID */
  {0, 0, 0, 0}, /* Roll  PID */
  {0, 0, 0, 0}, /* Yaw   PID */
};

/**
 * Default input mode settings.
 */
InputModeStruct g_modeSettings[3] = {
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-60,               /* Min angle */
   60,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_ANGLE}, /* Mode ID   */
  {-90,               /* Min angle */
   90,                /* Max angle */
   0,                 /* Offset    */
   20,                /* Speed     */
   INPUT_MODE_SPEED}, /* Mode ID   */
};

/**
 * Local variables
 */
static fix16_t camRot[3] = {0.0f};
static fix16_t camRotSpeedPrev[3] = {0.0f};

static fix16_t accelKp = 0;
static fix16_t accelKi = 0;

/* Accelerometer filter variables. */
static uint8_t fAccelFilterEnabled = TRUE;
static fix16_t accel_alpha = 0;

/* PID controller parameters. */
static PIDStruct PID[3] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

/**
 * @brief  Implements basic PID stabilization of the motor speed with feed forward.
 * @param  cmd_id - command id to apply PID action to.
 * @param  sp - set-point value.
 * @param  pv - process variable value.
 * @param  d - disturbance value from the 2nd IMU.
 * @return weighted sum of P, I and D actions.
 */
static float pidControllerApply(uint8_t cmd_id, fix16_t sp, fix16_t pv, fix16_t d) {
  fix16_t poles2 = fix16_from_int(g_pwmOutput[cmd_id].num_poles >> 1);
  /* Distance for the motor to travel: */
  fix16_t distance = circadjust(sp - pv, fix16_pi);
  /* Convert mechanical distance to electrical distance: */
  distance = fix16_mul(distance, poles2);
  /* If there is a distance to travel then rotate the motor in small steps: */
  fix16_t step = fix16_mul(distance, PID[cmd_id].I);
  step = constrain(step, MOTOR_STEP_LIMIT_MIN, MOTOR_STEP_LIMIT_MAX);
  /* Calculate proportional speed of the motor: */
  fix16_t speed = distance - PID[cmd_id].prevDistance;
  step += fix16_mul(speed, PID[cmd_id].P);
  /* Account for the acceleration of the motor: */
  step += fix16_mul(speed - PID[cmd_id].prevSpeed, PID[cmd_id].D);
  /* Account for the disturbance value (only if the 2nd IMU is enabled): */
  fix16_t disturbance = fix16_mul(PID[cmd_id].prevDisturbance - d, poles2);
  step += fix16_mul(disturbance, PID[cmd_id].F);
  /* Update offset of the motor: */
  g_motorOffset[cmd_id] += fix16_div(step, poles2);
  /* Wind-up guard limits motor offset range to one mechanical rotation: */
  g_motorOffset[cmd_id] = circadjust(g_motorOffset[cmd_id], fix16_pi);
  /* Update motor position: */
  float cmd = fix16_to_float(PID[cmd_id].prevCommand + step);
  /* Normalize command to -M_PI..M_PI range: */
  if (cmd < 0.0f) {
    cmd = fmodf(cmd - M_PI, -M_TWOPI) + M_PI;
  } else {
    cmd = fmodf(cmd + M_PI,  M_TWOPI) - M_PI;
  }
  /* Save values for the next iteration: */
  PID[cmd_id].prevDistance    = distance;
  PID[cmd_id].prevSpeed       = speed;
  PID[cmd_id].prevDisturbance = d;
  PID[cmd_id].prevCommand     = fix16_from_float(cmd);
  return cmd;
}

/**
 * @brief
 */
static void pidUpdateStruct(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    PID[i].P = fix16_mul(fix16_from_int(g_pidSettings[i].P), fix16_from_float(0.1f));
    PID[i].I = fix16_mul(fix16_from_int(g_pidSettings[i].I), fix16_from_float(0.01f));
    PID[i].D = fix16_mul(fix16_from_int(g_pidSettings[i].D), fix16_from_float(1.0f));
    PID[i].F = fix16_mul(fix16_from_int(g_pidSettings[i].F), fix16_from_float(0.01f));
    if (g_pidSettings[i].I == 0) {
      g_motorOffset[i] = 0;
    }
  }
}

/**
 * @brief  First order low-pass filter.
 * @param  raw - pointer to raw data array;
 * @param  filtered - pointer to filtered data array;
 */
static void accelFilterApply(const v3d *raw, v3d *filtered) {
  if (fAccelFilterEnabled) {
    v3d tmp;
    v3d_sub(&tmp, filtered, raw);
    v3d_mul_s(&tmp, &tmp, accel_alpha);
    v3d_add(filtered, &tmp, raw);
  } else {
    memcpy((void *)filtered, (void *)raw, sizeof(v3d));
  }
}

/**
 * @brief Find roll, pitch and yaw from quaternion.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static void Quaternion2RPY(const qf16 *q, v3d *rpy) {
  fix16_t R13, R11, R12, R23, R33;
  fix16_t qCs = fix16_mul(q->c, q->c);

  R11 = qCs + fix16_mul(q->d, q->d);
  R11 = fix16_one - fix16_mul(R11, fix16_from_int(2));
  R12 = fix16_mul(q->a, q->d) + fix16_mul(q->b, q->c);
  R12 = fix16_mul(R12, fix16_from_int(2));
  R13 = fix16_mul(q->a, q->c) - fix16_mul(q->b, q->d);
  R13 = fix16_mul(R13, fix16_from_int(2));
  R23 = fix16_mul(q->a, q->b) + fix16_mul(q->c, q->d);
  R23 = fix16_mul(R23, fix16_from_int(2));
  R33 = qCs + fix16_mul(q->b, q->b);
  R33 = fix16_one - fix16_mul(R33, fix16_from_int(2));

  rpy->y = fix16_asin (R13);   // roll always between -pi/2 to pi/2
  rpy->z = fix16_atan2(R12, R11);
  rpy->x = fix16_atan2(R23, R33);

  //TODO: consider the cases where |R13| ~= 1, |roll| ~= pi/2
}

/**
 * @brief
 */
void attitudeInit(void) {
  memset((void *)PID, 0, sizeof(PID));
  pidUpdateStruct();
  accelKp = fix16_from_float(0.02f / FIXED_DT_STEP);
  accelKi = fix16_from_float(0.0002f);
  accel_alpha = fix16_exp(fix16_from_float(-FIXED_DT_STEP / ACCEL_TAU));
}

/**
 * @brief
 */
void attitudeUpdate(PIMUStruct pIMU) {
  fix16_t mag;
  v3d accelErr = {0, 0, 0};
  v3d tmp;
  qf16 dq;

  /* Remove bias from accelerometer data. */
  v3d_sub(&pIMU->accelData, &pIMU->accelData, &pIMU->accelBias);
  /* Remove bias from gyroscope data. */
  v3d_sub(&pIMU->gyroData, &pIMU->gyroData, &pIMU->gyroBias);

  // Account for acceleration magnitude.
  mag = v3d_norm(&pIMU->accelData);

  if ((mag > GRAV_LIMIT_LOW) && (mag < GRAV_LIMIT_HIGH)) {
    // Rotate gravity to body frame and cross with accelerometer data.
    tmp.x = fix16_mul(pIMU->qIMU.b, pIMU->qIMU.d) - fix16_mul(pIMU->qIMU.a, pIMU->qIMU.c);
    tmp.y = fix16_mul(pIMU->qIMU.c, pIMU->qIMU.d) + fix16_mul(pIMU->qIMU.a, pIMU->qIMU.b);
    tmp.z = fix16_mul(pIMU->qIMU.b, pIMU->qIMU.b) + fix16_mul(pIMU->qIMU.c, pIMU->qIMU.c);
    v3d_mul_s(&tmp, &tmp, fix16_from_int(2));
    tmp.x = -tmp.x;
    tmp.y = -tmp.y;
    tmp.z = tmp.z - fix16_one;

    // Apply smoothing to accel values, to reduce vibration noise before main calculations.
    accelFilterApply(&pIMU->accelData, &pIMU->accelFiltered);
    // Apply the same filtering to the rotated attitude to match phase shift.
    accelFilterApply(&tmp, &pIMU->grotFiltered);
    // Compute the error between the predicted direction of gravity and smoothed acceleration.
    v3d_cross(&accelErr, &pIMU->accelFiltered, &pIMU->grotFiltered);

    // Normalize accel_error.
    v3d_div_s(&accelErr, &accelErr, mag);
  }

  // Correct rates based on error.
  v3d_mul_s(&tmp, &accelErr, accelKp);
  v3d_add(&pIMU->gyroData, &pIMU->gyroData, &tmp);

  // Correct rates based on error.
  v3d_mul_s(&tmp, &accelErr, accelKi);
  v3d_add(&pIMU->gyroBias, &pIMU->gyroBias, &tmp);

  dq.a = -fix16_mul(pIMU->qIMU.b, pIMU->gyroData.x) - fix16_mul(pIMU->qIMU.c, pIMU->gyroData.y) - fix16_mul(pIMU->qIMU.d, pIMU->gyroData.z);
  dq.b =  fix16_mul(pIMU->qIMU.a, pIMU->gyroData.x) - fix16_mul(pIMU->qIMU.d, pIMU->gyroData.y) + fix16_mul(pIMU->qIMU.c, pIMU->gyroData.z);
  dq.c =  fix16_mul(pIMU->qIMU.d, pIMU->gyroData.x) + fix16_mul(pIMU->qIMU.a, pIMU->gyroData.y) - fix16_mul(pIMU->qIMU.b, pIMU->gyroData.z);
  dq.d = -fix16_mul(pIMU->qIMU.c, pIMU->gyroData.x) + fix16_mul(pIMU->qIMU.b, pIMU->gyroData.y) + fix16_mul(pIMU->qIMU.a, pIMU->gyroData.z);
  qf16_mul_s(&dq, &dq, fix16_from_float(FIXED_DT_STEP*DEG2RAD*0.5f));

  /* Update attitude quaternion. */
  qf16_add(&pIMU->qIMU, &pIMU->qIMU, &dq);
  /* Normalize attitude quaternion. */
  qf16_normalize(&pIMU->qIMU, &pIMU->qIMU);

  /* Convert attitude into Euler angles. */
  Quaternion2RPY(&pIMU->qIMU, &pIMU->rpy);
}

/**
 * @brief
 */
void cameraRotationUpdate(void) {
  uint8_t i;
  fix16_t coef;
  fix16_t speedLimit;
  fix16_t tmp;

  for (i = 0; i < 3; i++) {
    speedLimit = fix16_deg_to_rad(fix16_from_int(g_modeSettings[i].speed));

    if (g_modeSettings[i].mode_id & INPUT_MODE_FOLLOW) {
      /* Calculate offset of the gimbal: */
      coef = fix16_deg_to_rad(fix16_from_int(g_modeSettings[i].offset)) - g_motorOffset[i];
      if (coef > MODE_FOLLOW_DEAD_BAND) {
        coef -= MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef = fix16_div(coef, fix16_from_float(INPUT_SIGNAL_ALPHA*FIXED_DT_STEP));
      } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
        coef += MODE_FOLLOW_DEAD_BAND;
        /* Convert to speed: */
        coef = fix16_div(coef, fix16_from_float(INPUT_SIGNAL_ALPHA*FIXED_DT_STEP));
      } else {
        coef = 0;
      }
    } else if (g_mixedInput[i].channel_id == INPUT_CHANNEL_DISABLED) {
      camRot[i] = 0;
      continue;
    } else {
      /* Calculate input scaling coefficient: */
      if (g_mixedInput[i].max_val == g_mixedInput[i].min_val) {
        /* Avoid divisions by zero. */
        coef = 0;
      } else {
        coef = fix16_div(fix16_from_int(g_inputValues[g_mixedInput[i].channel_id] - g_mixedInput[i].mid_val),
                         fix16_from_int(g_mixedInput[i].max_val - g_mixedInput[i].min_val));
      }

      if (g_modeSettings[i].mode_id & INPUT_MODE_SPEED) {
        /* Calculate speed from RC input data: */
        tmp = fix16_mul(speedLimit, fix16_from_int(2));
        coef = fix16_mul(coef, tmp);
        tmp = fix16_div(coef - camRotSpeedPrev[i], fix16_from_float(INPUT_SIGNAL_ALPHA));
        camRotSpeedPrev[i] += tmp;
        coef = camRotSpeedPrev[i];
      } else { /* INPUT_MODE_ANGLE */
        /* Calculate angle from input data: */
        coef = fix16_mul(coef, fix16_from_int(g_modeSettings[i].max_angle - g_modeSettings[i].min_angle));
        coef += fix16_div(fix16_from_int(g_modeSettings[i].max_angle + g_modeSettings[i].min_angle), fix16_from_int(2));
        coef = constrain(coef, fix16_from_int(g_modeSettings[i].min_angle), fix16_from_int(g_modeSettings[i].max_angle));
        coef = fix16_deg_to_rad(coef);

        /* Convert angle difference to speed: */
        coef = fix16_div(coef - camRot[i], fix16_from_float(INPUT_SIGNAL_ALPHA * FIXED_DT_STEP));
      }
    }
    coef = constrain(coef, -speedLimit, speedLimit);
    camRot[i] += fix16_mul(coef, fix16_from_float(FIXED_DT_STEP));
    camRot[i] = circadjust(camRot[i], fix16_pi);
  }
}

/**
 * @brief
 */
void actuatorsUpdate(void) {
  uint8_t i;
  for (i = 0; i < PWM_OUT_CMD_DISABLED; i++) {
    uint8_t cmd_id = g_pwmOutput[i].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
    float cmd = 0.0f;
    switch (cmd_id) {
      case PWM_OUT_CMD_PITCH:
        cmd = pidControllerApply(cmd_id, camRot[PWM_OUT_CMD_PITCH], g_IMU1.rpy.x, g_IMU2.rpy.x);
        break;
      case PWM_OUT_CMD_ROLL:
        cmd = pidControllerApply(cmd_id, camRot[PWM_OUT_CMD_ROLL], g_IMU1.rpy.y, g_IMU2.rpy.y);
        break;
      case PWM_OUT_CMD_YAW:
        cmd = pidControllerApply(cmd_id, camRot[PWM_OUT_CMD_YAW], g_IMU1.rpy.z, g_IMU2.rpy.z);
        break;
      default:;
    }
    pwmOutputUpdate(i, cmd);
  }
}

/**
 * @brief
 */
void pidSettingsUpdate(const PPIDSettings pNewSettings) {
  memcpy((void *)&g_pidSettings, (void *)pNewSettings, sizeof(g_pidSettings));
  pidUpdateStruct();
}

/**
 * @brief
 */
void inputModeSettingsUpdate(const PInputModeStruct pNewSettings) {
  memcpy((void *)&g_modeSettings, (void *)pNewSettings, sizeof(g_modeSettings));
}
