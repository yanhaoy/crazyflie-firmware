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
  // setpoint_t setpoint_att;
  // positionController(&actuatorThrust, &attitudeDesired, &setpoint_att, state);

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
  double xd[12] = {0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // double xd[12] = {0, 0, 0.2, attitudeDesired.roll, -attitudeDesired.pitch, 0, 0, 0, 0, 0, 0, 0};
  double u[4];

  static const double a[48] = {
      -1.42109374580397E-10, 1.3174886923822E-10,   7.9473403046769E-11,
      -6.91198985445977E-11, 1.23271749001992E-10,  8.80030660879707E-11,
      -9.83275437179508E-11, -1.12943574185428E-10, 0.501038320386816,
      0.501038320379042,     0.501038320373577,     0.501038320389357,
      -0.00174840069801478,  -0.00124816980883738,  0.00139463138368215,
      0.00160193694071126,   0.0020155953693786,    -0.00186865067023277,
      -0.00112720461679378,  0.000980353993022433,  -1.36730615838381E-8,
      1.35339205295553E-8,   -1.31861815860936E-8,  1.33253226133042E-8,
      -2.41653774545027E-7,  2.24036175750219E-7,   1.35142814519028E-7,
      -1.17536703041075E-7,  2.09619299433809E-7,   1.49645678316361E-7,
      -1.6720520317817E-7,   -1.92059477013671E-7,  0.60536012048504,
      0.605360120484889,     0.605360120484858,     0.60536012048507,
      -0.08554915029891,     -0.0610723286640263,   0.0682388260061124,
      0.0783826529716936,    -0.0986221983635827,   0.0914344751742942,
      0.0551544725562608,    -0.0479667495255518,   -0.366271989046446,
      0.362544683409436,     -0.353229502083258,    0.356956807720255};
  double b_x[12];
  double d;
  int i;
  int k;
  /*  max_diff = [ */
  /*      0.05423; */
  /*      0.05423;  */
  /*      0.03792; */
  /*      1e1*0.007495; */
  /*      1e1*0.007495; */
  /*      0; */
  /*      0.02342; */
  /*      0.02342; */
  /*      0.03169; */
  /*      1e-1*0.1441; */
  /*      1e-1*0.1441; */
  /*      0.03733]; */
  /*  diff((abs(diff) > max_diff)) = sign(diff((abs(diff) >
   * max_diff)))*max_diff((abs(diff) > max_diff)); */
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

  if (count < 7.5e3 && count > 5e3)
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

  // motorPower.m1 = (uint32_t)(attitudeDesired.roll+360)*90;
  // motorPower.m2 = (uint32_t)(attitudeDesired.pitch+360)*90;
  // motorPower.m3 = (uint32_t)(state->attitude.roll+360)*90;
  // motorPower.m4 = (uint32_t)(state->attitude.pitch+360)*90;

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
