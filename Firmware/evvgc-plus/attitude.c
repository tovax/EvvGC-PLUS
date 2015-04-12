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

#include "attitude.h"
#include "misc.h"
#include "pwmio.h"

/**
 * Single precision floating point constants.
 */
#ifndef M_PI
#define M_PI                    ( 3.14159265f )
#endif

#define RAD2DEG                 ( 180.0f / M_PI )
#define DEG2RAD                 ( M_PI / 180.0f )

#define FIXED_DT_STEP           ( 0.0015f )

#define ACCEL_TAU               ( 0.1f )
#define INPUT_SIGNAL_ALPHA      ( 266.7f )

/**
 * Fixed point constants.
 */
#define MODE_FOLLOW_DEAD_BAND   F16( M_PI / 36.0f )

#define MOTOR_STEP_LIMIT_MAX    F16( M_PI / 45.0f )
#define MOTOR_STEP_LIMIT_MIN    F16( M_PI /-45.0f )

#define GRAV_LIMIT_LOW          F16( GRAV * 0.8f )
#define GRAV_LIMIT_HIGH         F16( GRAV * 1.2f )

#define PID_COEF_SCALE_P        F16( 0.1f )
#define PID_COEF_SCALE_I        F16( 0.01f )
#define PID_COEF_SCALE_D        F16( 1.0f )
#define PID_COEF_SCALE_F        F16( 0.01f )

static const fix16_t fix16_half = 0x00008000; /*!< fix16_t value of 0.5. */
static const fix16_t fix16_two  = 0x00020000; /*!< fix16_t value of 2. */

/* PID controller structure. */
typedef struct tagPIDStruct {
  fix16_t P;
  fix16_t I;
  fix16_t D;
  fix16_t F;
  fix16_t prevDistance;
  fix16_t prevSpeed;
  fix16_t prevCommand;
} __attribute__((packed)) PIDStruct, *PPIDStruct;

/**
 * Global variables.
 */
/* Mechanical offset of the motors. */
fix16_t g_motorOffset[3] = {0};

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
static v3d camAtti = {0};
static fix16_t camRot[3] = {0};
static fix16_t camRotSpeedPrev[3] = {0};
#if !defined(USE_ONE_IMU)
static v3d rpyIMU2Prev = {0};
#endif /* USE_ONE_IMU */

static fix16_t accel2Kp = 0;
static fix16_t accel2Ki = 0;

/* Accelerometer filter variables. */
static uint8_t fAccelFilterEnabled = TRUE;
static fix16_t accel_alpha = 0;

/* PID controller parameters. */
static PIDStruct PID[3] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0}
};

/**
 * @brief  Implements basic PID stabilization of the motor speed
 *         with feed forward (if the 2nd IMU is enabled).
 * @param  ch_id - channel id to apply PID action to.
 * @param  err - process error.
 * @param  rot - rotation command value.
 * @param  db - disturbance value from the 2nd IMU.
 * @return weighted sum of P, I and D actions.
 */
static float pidControllerApply(uint8_t ch_id, fix16_t err, fix16_t rot, fix16_t db) {
  fix16_t poles2 = fix16_from_int(g_pwmOutput[ch_id].num_poles / 2);
  /* Error is a distance for the motor to travel: */
  fix16_t distance = circadjust(err, fix16_pi);
  /* Convert mechanical distance to electrical distance: */
  distance = fix16_mul(distance, poles2);
  /* If there is a distance to travel then rotate the motor in small steps: */
  fix16_t step = fix16_mul(distance, PID[ch_id].I);
  step = constrain(step, MOTOR_STEP_LIMIT_MIN, MOTOR_STEP_LIMIT_MAX);
  /* Calculate proportional speed of the motor: */
  fix16_t speed = fix16_sub(distance, PID[ch_id].prevDistance);
  step = fix16_add(step, fix16_mul(speed, PID[ch_id].P));
  /* Account for the acceleration of the motor: */
  fix16_t acceleration = fix16_sub(speed, PID[ch_id].prevSpeed);
  step = fix16_add(step, fix16_mul(acceleration, PID[ch_id].D));
  /* Account for rotation command value: */
  step = fix16_add(step, fix16_mul(rot, poles2));
  /* Account for the disturbance value (only if the 2nd IMU is enabled): */
  db = circadjust(db, fix16_pi);
  /* Convert mechanical disturbance to electrical disturbance: */
  db = fix16_mul(db, poles2);
  step = fix16_add(step, fix16_mul(db, PID[ch_id].F));
  /* Update offset of the motor: */
  g_motorOffset[ch_id] = fix16_add(g_motorOffset[ch_id], fix16_div(step, poles2));
  /* Wind-up guard limits motor offset range to one mechanical rotation: */
  g_motorOffset[ch_id] = circadjust(g_motorOffset[ch_id], fix16_pi);
  /* Update motor position: */
  fix16_t cmd = fix16_add(PID[ch_id].prevCommand, step);
  /* Normalize command to -M_PI..M_PI range: */
  if (cmd < -fix16_pi) {
    do {
      cmd = fix16_add(cmd, fix16_two_pi);
    } while (cmd < -fix16_pi);
  } else if (cmd > fix16_pi) {
    do {
      cmd = fix16_sub(cmd, fix16_two_pi);
    } while (cmd > fix16_pi);
  }
  /* Save values for the next iteration: */
  PID[ch_id].prevDistance = distance;
  PID[ch_id].prevSpeed    = speed;
  PID[ch_id].prevCommand  = cmd;
  return cmd;
}

/**
 * @brief
 */
static void pidUpdateStruct(void) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    PID[i].P = fix16_mul(fix16_from_int(g_pidSettings[i].P), PID_COEF_SCALE_P);
    PID[i].I = fix16_mul(fix16_from_int(g_pidSettings[i].I), PID_COEF_SCALE_I);
    PID[i].D = fix16_mul(fix16_from_int(g_pidSettings[i].D), PID_COEF_SCALE_D);
    PID[i].F = fix16_mul(fix16_from_int(g_pidSettings[i].F), PID_COEF_SCALE_F);
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
static void qf16_to_rpy(const qf16 *q, v3d *rpy) {
  fix16_t R13, R11, R12, R23, R33;
  fix16_t qCs = fix16_mul(q->c, q->c);

  R11 = fix16_add(qCs, fix16_mul(q->d, q->d));
  R11 = fix16_sub(fix16_one, fix16_mul(R11, fix16_two));
  R12 = fix16_mul(q->a, q->d);
  R12 = fix16_add(R12, fix16_mul(q->b, q->c));
  R12 = fix16_mul(R12, fix16_two);
  R13 = fix16_mul(q->a, q->c);
  R13 = fix16_sub(R13, fix16_mul(q->b, q->d));
  R13 = fix16_mul(R13, fix16_two);
  R23 = fix16_mul(q->a, q->b);
  R23 = fix16_add(R23, fix16_mul(q->c, q->d));
  R23 = fix16_mul(R23, fix16_two);
  R33 = fix16_add(qCs, fix16_mul(q->b, q->b));
  R33 = fix16_sub(fix16_one, fix16_mul(R33, fix16_two));

  rpy->y = fix16_asin (R13);   // roll always between -pi/2 to pi/2
  rpy->z = fix16_atan2(R12, R11);
  rpy->x = fix16_atan2(R23, R33);

  //TODO: consider the cases where |R13| ~= 1, |roll| ~= pi/2
}
#if 0
/**
 * @brief Find quaternion from roll, pitch and yaw.
 * @note  The order of rotations is:
 *        1. pitch (X);
 *        2. roll (Y);
 *        3. yaw (Z).
 */
static void qf16_from_rpy(const v3d *rpy, qf16 *q) {
  fix16_t phi, theta, psi;
  fix16_t cphi, sphi, ctheta, stheta, cpsi, spsi;
  fix16_t tmp1, tmp2;

  phi    = fix16_div(rpy->x, fix16_two);
  theta  = fix16_div(rpy->y, fix16_two);
  psi    = fix16_div(rpy->z, fix16_two);

  cphi   = fix16_cos(phi);
  sphi   = fix16_sin(phi);
  ctheta = fix16_cos(theta);
  stheta = fix16_sin(theta);
  cpsi   = fix16_cos(psi);
  spsi   = fix16_sin(psi);

  tmp1 = fix16_mul(cphi, fix16_mul(ctheta, cpsi));
  tmp2 = fix16_mul(sphi, fix16_mul(stheta, spsi));
  q->a = fix16_add(tmp1, tmp2);
  tmp1 = fix16_mul(sphi, fix16_mul(ctheta, cpsi));
  tmp2 = fix16_mul(cphi, fix16_mul(stheta, spsi));
  q->b = fix16_sub(tmp1, tmp2);
  tmp1 = fix16_mul(cphi, fix16_mul(stheta, cpsi));
  tmp2 = fix16_mul(sphi, fix16_mul(ctheta, spsi));
  q->c = fix16_add(tmp1, tmp2);
  tmp1 = fix16_mul(cphi, fix16_mul(ctheta, spsi));
  tmp2 = fix16_mul(sphi, fix16_mul(stheta, cpsi));
  q->d = fix16_sub(tmp1, tmp2);
}
#endif /* 0 */

/**
 * @brief  Rotates vector v by rotation quaternion q.
 * @param  r - rotated vector.
 * @param  v - vector to be rotated.
 * @param  q - rotation quaternion.
 */
static void v3d_rot(v3d *r, const v3d *v, const qf16 *q) {
  fix16_t tmp;

  tmp = fix16_sub(fix16_half, fix16_mul(q->c, q->c));
  tmp = fix16_sub(tmp, fix16_mul(q->d, q->d));
  r->x = fix16_mul(tmp, v->x);
  tmp = fix16_sub(fix16_mul(q->b, q->c), fix16_mul(q->a, q->d));
  r->x = fix16_add(r->x, fix16_mul(tmp, v->y));
  tmp = fix16_add(fix16_mul(q->b, q->d), fix16_mul(q->a, q->c));
  r->x = fix16_add(r->x, fix16_mul(tmp, v->z));

  tmp = fix16_sub(fix16_half, fix16_mul(q->b, q->b));
  tmp = fix16_sub(tmp, fix16_mul(q->d, q->d));
  r->y = fix16_mul(tmp, v->y);
  tmp = fix16_sub(fix16_mul(q->c, q->d), fix16_mul(q->a, q->b));
  r->y = fix16_add(r->y, fix16_mul(tmp, v->z));
  tmp = fix16_add(fix16_mul(q->b, q->c), fix16_mul(q->a, q->d));
  r->y = fix16_add(r->y, fix16_mul(tmp, v->x));

  tmp = fix16_sub(fix16_half, fix16_mul(q->b, q->b));
  tmp = fix16_sub(tmp, fix16_mul(q->c, q->c));
  r->z = fix16_mul(tmp, v->z);
  tmp = fix16_sub(fix16_mul(q->b, q->d), fix16_mul(q->a, q->c));
  r->z = fix16_add(r->z, fix16_mul(tmp, v->x));
  tmp = fix16_add(fix16_mul(q->c, q->d), fix16_mul(q->a, q->b));
  r->z = fix16_add(r->z, fix16_mul(tmp, v->y));

  v3d_mul_s(r, r, fix16_two);
}

/**
 * @brief
 */
void attitudeInit(void) {
  pidUpdateStruct();
  accel2Kp = fix16_from_float(30.0f);
  accel2Ki = fix16_from_float(0.001f);
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

  /* Account for acceleration magnitude. */
  mag = v3d_norm(&pIMU->accelData);

  if ((mag > GRAV_LIMIT_LOW) && (mag < GRAV_LIMIT_HIGH)) {
    // Get estimated gravity vector halved and cross with accelerometer data.
    tmp.x = fix16_mul(pIMU->qIMU.a, pIMU->qIMU.c);
    tmp.x = fix16_sub(tmp.x, fix16_mul(pIMU->qIMU.b, pIMU->qIMU.d));
    tmp.y = fix16_mul(pIMU->qIMU.a, pIMU->qIMU.b);
    tmp.y = fix16_add(tmp.y, fix16_mul(pIMU->qIMU.c, pIMU->qIMU.d));
    tmp.y = -tmp.y;
    tmp.z = fix16_mul(pIMU->qIMU.b, pIMU->qIMU.b);
    tmp.z = fix16_add(tmp.z, fix16_mul(pIMU->qIMU.c, pIMU->qIMU.c));
    tmp.z = fix16_sub(tmp.z, fix16_half);

    // Apply smoothing to accel values, to reduce vibration noise before main calculations.
    accelFilterApply(&pIMU->accelData, &pIMU->accelFiltered);
    // Apply the same filtering to the rotated attitude to match phase shift.
    accelFilterApply(&tmp, &pIMU->v2Filtered);
    // Compute the error between the predicted direction of gravity and smoothed acceleration.
    v3d_cross(&accelErr, &pIMU->accelFiltered, &pIMU->v2Filtered);

    // Normalize accel_error.
    v3d_div_s(&accelErr, &accelErr, mag);
  }

  // Correct rates based on error.
  v3d_mul_s(&tmp, &accelErr, accel2Kp);
  v3d_add(&pIMU->gyroData, &pIMU->gyroData, &tmp);

  // Correct bias based on error.
  v3d_mul_s(&tmp, &accelErr, accel2Ki);
  v3d_sub(&pIMU->gyroBias, &pIMU->gyroBias, &tmp);

  /* Remove bias from gyroscope data. */
  v3d_sub(&pIMU->gyroData, &pIMU->gyroData, &pIMU->gyroBias);

  // Calculate derivative of the attitude quaternion.
  dq.a = fix16_mul(pIMU->qIMU.b, pIMU->gyroData.x);
  dq.a = fix16_add(dq.a, fix16_mul(pIMU->qIMU.c, pIMU->gyroData.y));
  dq.a = fix16_add(dq.a, fix16_mul(pIMU->qIMU.d, pIMU->gyroData.z));
  dq.a = -dq.a;
  dq.b = fix16_mul(pIMU->qIMU.a, pIMU->gyroData.x);
  dq.b = fix16_sub(dq.b, fix16_mul(pIMU->qIMU.d, pIMU->gyroData.y));
  dq.b = fix16_add(dq.b, fix16_mul(pIMU->qIMU.c, pIMU->gyroData.z));
  dq.c = fix16_mul(pIMU->qIMU.d, pIMU->gyroData.x);
  dq.c = fix16_add(dq.c, fix16_mul(pIMU->qIMU.a, pIMU->gyroData.y));
  dq.c = fix16_sub(dq.c, fix16_mul(pIMU->qIMU.b, pIMU->gyroData.z));
  dq.d = fix16_mul(pIMU->qIMU.c, pIMU->gyroData.x);
  dq.d = fix16_sub(dq.d, fix16_mul(pIMU->qIMU.b, pIMU->gyroData.y));
  dq.d = fix16_sub(dq.d, fix16_mul(pIMU->qIMU.a, pIMU->gyroData.z));
  dq.d = -dq.d;
  qf16_mul_s(&dq, &dq, fix16_from_float(FIXED_DT_STEP*DEG2RAD*0.5f));

  /* Update attitude quaternion. */
  qf16_add(&pIMU->qIMU, &pIMU->qIMU, &dq);
  /* Normalize attitude quaternion. */
  qf16_normalize(&pIMU->qIMU, &pIMU->qIMU);
  /* Convert to Euler angles. */
  qf16_to_rpy(&pIMU->qIMU, &pIMU->rpyIMU);
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
      coef = fix16_deg_to_rad(fix16_from_int(g_modeSettings[i].offset));
      coef = fix16_sub(coef, g_motorOffset[i]);
      if (coef > MODE_FOLLOW_DEAD_BAND) {
        coef = fix16_sub(coef, MODE_FOLLOW_DEAD_BAND);
        /* Convert to speed: */
        coef = fix16_div(coef, fix16_from_float(INPUT_SIGNAL_ALPHA*FIXED_DT_STEP));
      } else if (coef < -MODE_FOLLOW_DEAD_BAND) {
        coef = fix16_add(coef, MODE_FOLLOW_DEAD_BAND);
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
        tmp  = fix16_mul(speedLimit, fix16_two);
        coef = fix16_mul(coef, tmp);
        tmp  = fix16_div(fix16_sub(coef, camRotSpeedPrev[i]), fix16_from_float(INPUT_SIGNAL_ALPHA));
        camRotSpeedPrev[i] = fix16_add(camRotSpeedPrev[i], tmp);
        coef = camRotSpeedPrev[i];
      } else { /* INPUT_MODE_ANGLE */
        /* Calculate angle from input data: */
        coef = fix16_mul(coef, fix16_from_int(g_modeSettings[i].max_angle - g_modeSettings[i].min_angle));
        tmp  = fix16_div(fix16_from_int(g_modeSettings[i].max_angle + g_modeSettings[i].min_angle), fix16_two);
        coef = fix16_add(coef, tmp);
        coef = constrain(coef, fix16_from_int(g_modeSettings[i].min_angle), fix16_from_int(g_modeSettings[i].max_angle));
        coef = fix16_deg_to_rad(coef);

        /* Convert angle difference to speed: */
        coef = fix16_div(fix16_sub(coef, camRot[i]), fix16_from_float(INPUT_SIGNAL_ALPHA*FIXED_DT_STEP));
      }
    }
    coef = constrain(coef, -speedLimit, speedLimit);
    camRot[i] = fix16_mul(coef, fix16_from_float(FIXED_DT_STEP));
  }
}

/**
 * @brief
 */
void actuatorsUpdate(void) {
  float cmd = 0.0f;
  v3d err;
  v3d db;

  /* Find error of the process. */
  v3d_sub(&err, &camAtti, &g_IMU1.rpyIMU);
  /* Update attitude of the camera by amount of commanded rotation. */
  v3d_add(&camAtti, &camAtti, (v3d *)camRot);

  camAtti.x = circadjust(camAtti.x, fix16_pi);
  camAtti.y = circadjust(camAtti.y, fix16_pi);
  camAtti.z = circadjust(camAtti.z, fix16_pi);

#if !defined(USE_ONE_IMU)
  v3d diff;
  /* Compute disturbance value. */
  v3d_sub(&diff, &rpyIMU2Prev, &g_IMU2.rpyIMU);
  /* Rotate disturbance. */
  v3d_rot(&db, &diff, &g_IMU2.qIMU);
  /* Store attitude value of the second IMU. */
  memcpy((void *)&rpyIMU2Prev, (void *)&g_IMU2.rpyIMU, sizeof(rpyIMU2Prev));
#else
  memset((void *)&db, 0, sizeof(db));
#endif /* USE_ONE_IMU */

  fix16_t *p1 = (fix16_t *)&err;
  fix16_t *p2 = (fix16_t *)&db;

  /* Pitch: */
  uint8_t ch_id = g_pwmOutput[PWM_OUT_PITCH].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (ch_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(ch_id, p1[ch_id], camRot[ch_id], p2[ch_id]);
  }
  pwmOutputUpdate(PWM_OUT_PITCH, cmd);
  cmd = 0.0f;
  /* Roll: */
  ch_id = g_pwmOutput[PWM_OUT_ROLL].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (ch_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(ch_id, p1[ch_id], camRot[ch_id], p2[ch_id]);
  }
  pwmOutputUpdate(PWM_OUT_ROLL, cmd);
  cmd = 0.0f;
  /* Yaw: */
  ch_id = g_pwmOutput[PWM_OUT_YAW].dt_cmd_id & PWM_OUT_CMD_ID_MASK;
  if (ch_id != PWM_OUT_CMD_DISABLED) {
    cmd = pidControllerApply(ch_id, p1[ch_id], camRot[ch_id], p2[ch_id]);
  }
  pwmOutputUpdate(PWM_OUT_YAW, cmd);
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
