/**
  ******************************************************************************
  * @file           : mag.h
  * @brief          : Contains API functions for rev 3's BMM350 magnetometer.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  */

/* How it works, studying the BMM350 driver:
 * - You read the 32 byte OTP ROM on startup
 * - You do an I2C burst read of X, Y, Z, and temp (used for compensation)
 * - You read compensation coefficents from the OTP ROM
 * - You apply them and pass back the result
 */

/*------------------------------------------------------------------------------
 Typdefs
------------------------------------------------------------------------------*/

// TODO this needs to be expanded
// Currently just the minimum for this to be kind of useful as return
typedef enum _MAG_STATUS
    {
    MAG_OK = 0,
    MAG_FAIL
    } MAG_STATUS;

/* Set pad driver strength (See BMM350 Datasheet section 5.6) */
typedef enum _MAG_PAD_CTRL
    {
    MAG_PAD_CTRL_0       = 0x0,
    MAG_PAD_CTRL_1       = 0x1,
    MAG_PAD_CTRL_2       = 0x2,
    MAG_PAD_CTRL_3       = 0x3,
    MAG_PAD_CTRL_4       = 0x4,
    MAG_PAD_CTRL_5       = 0x5,
    MAG_PAD_CTRL_6       = 0x6,
    MAG_PAD_CTRL_7       = 0x7,
    MAG_PAD_CTRL_DEFAULT = 0x7 /* Same as MAG_PAD_CTRL_7 */
    } MAG_PAD_CTRL;

/* Sensor sampling rates */
typedef enum _MAG_ODR
    {
    MAG_ODR_400HZ    = 0x2,
    MAG_ODR_200HZ    = 0x3,
    MAG_ODR_100HZ    = 0x4,
    MAG_ODR_50HZ     = 0x5,
    MAG_ODR_25HZ     = 0x6,
    MAG_ODR_12_5HZ   = 0x7,
    MAG_ODR_6_25HZ   = 0x8,
    MAG_ODR_3_125HZ  = 0x9,
    MAG_ODR_1_5625HZ = 0xa
    } MAG_ODR;

/* Sensor measurement averaging settings */
typedef enum _MAG_AVG
    {
    MAG_AVG_0 = 0x0, /* No averaging */
    MAG_AVG_2 = 0x1, /* Of two samples */
    MAG_AVG_4 = 0x2, /* Of four samples */
    MAG_AVG_8 = 0x3, /* Of eight samples */
    } MAG_AVG;

/* Axis enablement settings */
typedef enum _MAG_AXIS_X
    {
    MAG_AXIS_X_DISABLED = 0,
    MAG_AXIS_X_ENABLED = 1
    } MAG_AXIS_X;

typedef enum _MAG_AXIS_Y
    {
    MAG_AXIS_Y_DISABLED = 0,
    MAG_AXIS_Y_ENABLED = 1
    } MAG_AXIS_Y;

typedef enum _MAG_AXIS_Z
    {
    MAG_AXIS_Z_DISABLED = 0,
    MAG_AXIS_Z_ENABLED = 1
    } MAG_AXIS_Z;

/* Magnetometer config struct */
typedef struct _MAG_CONFIG
    {
    MAG_PAD_CTRL pad_ctrl;

    /* Sampling settings */
    MAG_ODR odr;
    MAG_AVG avg;

    /* Axis Enablement Settings */
    MAG_AXIS_X axis_x;
    MAG_AXIS_Y axis_y;
    MAG_AXIS_Z axis_z;
    } MAG_CONFIG;

/* Compensated Magnetomer Data struct */
typedef struct _MAG_XYZ
    {
    float mag_x;
    float mag_y;
    float mag_z;
    } MAG_XYZ;

/*------------------------------------------------------------------------------
    Function Prototypes
------------------------------------------------------------------------------*/
MAG_STATUS mag_init
    (
    MAG_CONFIG *mag_config_ptr
    );

MAG_STATUS mag_get_ready
    (
    void
    );

MAG_STATUS mag_it_handler
    (
    void
    );

MAG_STATUS mag_get_xyz
    (
    MAG_XYZ *mag_xyz
    );
