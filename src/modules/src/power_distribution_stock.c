/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "position_controller.h"
#include <math.h>

long long count = 0;

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control, setpoint_t *setpoint, const sensorData_t *sensorData, const state_t *state)
{
  // attitude_t attitudeDesired;
  // float actuatorThrust;
  // positionController(&actuatorThrust, &attitudeDesired, setpoint, state);

  double x[12] = {
      (double)state->position.x,
      (double)state->position.y,
      (double)state->position.z,
      (double)state->attitude.roll / 180 * 3.14159,
      (double)state->attitude.pitch / 180 * 3.14159,
      (double)state->attitude.yaw / 180 * 3.14159,
      (double)state->velocity.x,
      (double)state->velocity.y,
      (double)state->velocity.z,
      (double)sensorData->gyro.x / 180 * 3.14159,
      (double)sensorData->gyro.y / 180 * 3.14159,
      (double)sensorData->gyro.z / 180 * 3.14159,
  };
  // double xd[12] = {0.1, attitudeDesired.roll, attitudeDesired.pitch, 0, 0, 0, 0, 0};
  double xd[12] = {0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double u[4];

  static const double a[48] = {
      -5.42031522336395E-11, 5.28285658101859E-11,  4.78714870443311E-11,
      -4.64969006374907E-11, 5.23770986715285E-11,  4.85239631466303E-11,
      -4.98922458764217E-11, -5.10088159717303E-11, 0.136251652900625,
      0.136251652898964,     0.136251652900705,     0.136251652898954,
      -0.137837509305527,    -0.127697435211802,    0.131298262056926,
      0.134236682460475,     0.142643032059928,     -0.13902561619545,
      -0.12598034599679,     0.122362930132341,     -0.166318754761903,
      0.170380581138883,     -0.180246047253006,    0.176184220872694,
      -1.25550384916621E-6,  1.22366438583222E-6,   1.10884374914901E-6,
      -1.07700428623403E-6,  1.21320703591205E-6,   1.12395704134225E-6,
      -1.15565049302598E-6,  -1.18151358519502E-6,  0.0553877711950844,
      0.0553877711948437,    0.0553877711950973,    0.0553877711948432,
      -0.0351830958017375,   -0.0323624366537977,   0.0333202309115897,
      0.0342253015439436,    -0.0365574204424846,   0.0355953176475468,
      0.0318200540569092,    -0.0308579512619764,   -0.0463173892937005,
      0.0472859766806133,    -0.0496349272963494,   0.0486663399093464};
  double b_x[12];
  double d;
  int i;
  int k;
  for (i = 0; i < 12; i++) {
    b_x[i] = x[i] - xd[i];
  }
  for (k = 0; k < 4; k++) {
    d = 0.0;
    for (i = 0; i < 12; i++) {
      d += a[k + (i << 2)] * b_x[i];
    }
    u[k] = (sqrt(1.066330912689E-12 -
                 8.5211799999999988E-11 *
                     (0.000548456 - (0.068670000000000009 - d))) +
            -1.032633E-6) /
           4.2605899999999994E-11;
  }

  for (size_t i = 0; i < 4; i++)
  {
    u[i] = limitThrust(u[i]);
  }

  if (count < 15e3 && count > 5e3)
  {
    motorsSetRatio(MOTOR_M1, u[0]);
    motorsSetRatio(MOTOR_M2, u[1]);
    motorsSetRatio(MOTOR_M3, u[2]);
    motorsSetRatio(MOTOR_M4, u[3]);

    motorPower.m1 = u[0];
    motorPower.m2 = u[1];
    motorPower.m3 = u[2];
    motorPower.m4 = u[3];
  }
  else
  {
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);

    motorPower.m1 = 0;
    motorPower.m2 = 0;
    motorPower.m3 = 0;
    motorPower.m4 = 0;
  }

  // motorsSetRatio(MOTOR_M1, 0);
  // motorsSetRatio(MOTOR_M2, 0);
  // motorsSetRatio(MOTOR_M3, 0);
  // motorsSetRatio(MOTOR_M4, 0);

  // motorPower.m1 = u[0];
  // motorPower.m2 = u[1];
  // motorPower.m3 = u[2];
  // motorPower.m4 = u[3];

  count++;
}

/**
 * Override power distribution to motors.
 */
PARAM_GROUP_START(motorPowerSet)

/**
 * @brief Nonzero to override controller with set values
 */
PARAM_ADD_CORE(PARAM_UINT8, enable, &motorSetEnable)

/**
 * @brief motor power for m1: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m1, &motorPowerSet.m1)

/**
 * @brief motor power for m2: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m2, &motorPowerSet.m2)

/**
 * @brief motor power for m3: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m3, &motorPowerSet.m3)

/**
 * @brief motor power for m4: `0 - UINT16_MAX`
 */
PARAM_ADD_CORE(PARAM_UINT16, m4, &motorPowerSet.m4)

PARAM_GROUP_STOP(motorPowerSet)

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 * Motor output related log variables.
 */
LOG_GROUP_START(motor)
/**
 * @brief Motor power (PWM value) for M1 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m1, &motorPower.m1)
/**
 * @brief Motor power (PWM value) for M2 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m2, &motorPower.m2)
/**
 * @brief Motor power (PWM value) for M3 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m3, &motorPower.m3)
/**
 * @brief Motor power (PWM value) for M4 [0 - UINT16_MAX]
 */
LOG_ADD_CORE(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
