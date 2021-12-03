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
#include <stdlib.h>

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
  // Runs at 500hz
  count++;

  // States
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

  // Hovering height
  double zd;
  if (count < 15e3)
  {
    zd = 0.5;
  }
  else
  {
    zd = 0.25;
  }
  double dz = zd - x[2];
  if (abs(dz) > 0.05)
  {
    zd = x[2] + dz / abs(dz) * 0.05;
  }

  // Desired states
  double xd[12] = {0, 0, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Controls
  double u[4];

  // LQR
  static const double a[48] = {
      -0.0111650626931185, 0.0111650626931062, 0.0111650626930822,
      -0.0111650626931557, 0.0111650626931383, 0.0111650626931322,
      -0.0111650626931444, -0.0111650626931354, 0.160365685385961,
      0.160365685386052, 0.160365685385978, 0.160365685385899,
      -0.0275715937900411, -0.0275715937900256, 0.0275715937900418,
      0.0275715937900276, 0.0275715937900451, -0.0275715937899942,
      -0.0275715937899778, 0.027571593790037, -0.0639185163509148,
      0.0639185163509139, -0.0639185163508666, 0.063918516350876,
      -0.00792634823244657, 0.00792634823242948, 0.00792634823242092,
      -0.00792634823245076, 0.00792634823244899, 0.00792634823244369,
      -0.00792634823245077, -0.00792634823244466, 0.197452751726173,
      0.197452751726109, 0.197452751726102, 0.197452751726184,
      -0.00510298076889898, -0.00510298076889909, 0.00510298076889899,
      0.00510298076889914, -0.00510298076889968, 0.00510298076889819,
      0.00510298076889786, -0.00510298076889965, -0.035357366737476,
      0.0353573667374777, -0.0353573667374742, 0.0353573667374761};
  double b_x[12];
  double d;
  int i;
  int k;
  /*  Real */
  /*  Sim */
  /*  a = 9.2416e-10; */
  /*  b = 2.9342e-06; */
  /*  c = 0.0023 - u; */
  for (i = 0; i < 12; i++)
  {
    b_x[i] = x[i] - xd[i];
  }
  for (k = 0; k < 4; k++)
  {
    d = 0.0;
    for (i = 0; i < 12; i++)
    {
      d += a[k + (i << 2)] * b_x[i];
    }
    u[k] = (sqrt(1.066330912689E-12 -
                 8.5211799999999988E-11 * (0.000548456 - (0.0662175 - d))) +
            -1.032633E-6) /
           4.2605899999999994E-11;
  }

  for (size_t i = 0; i < 4; i++)
  {
    u[i] = limitThrust(u[i]);
  }

  if (count < 17.5e3 && count > 2.5e3)
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
