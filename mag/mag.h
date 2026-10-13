/**
  ******************************************************************************
  * @file           : mag.h
  * @brief          : Contains interface for magnetometer
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAG_H
#define MAG_H

#ifdef __cplusplus
extern "C" {
#endif

// #include "stm32h7xx_hal.h"

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
// #ifdef UNIT_TEST
// 	#include <stdint.h>
// #endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

typedef struct _MAG_CONVERTED
    {
    float mag_x;
    float mag_y;
    float mag_z;
    } MAG_CONVERTED;

typedef struct _MAG_RAW
    {
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    uint16_t mag_hall;
    } MAG_RAW;

/** @brief Magnetometer output data rate, written to MAG_REG_CTRL1           */
typedef enum _MAG_ODR_SETTING
    {
    MAG_ODR_10HZ = ( 0b000 << 3 ),
    MAG_ODR_2HZ  = ( 0b001 << 3 ),
    MAG_ODR_6HZ  = ( 0b010 << 3 ),
    MAG_ODR_8HZ  = ( 0b011 << 3 ),
    MAG_ODR_15HZ = ( 0b100 << 3 ),
    MAG_ODR_20HZ = ( 0b101 << 3 ),
    MAG_ODR_25HZ = ( 0b110 << 3 ),
    MAG_ODR_30HZ = ( 0b111 << 3 )
    } MAG_ODR_SETTING;

/** @brief Magnetometer operating mode, written to MAG_REG_CTRL1             */
typedef enum _MAG_OP_MODE
    {
    MAG_NORMAL_MODE = ( 0b00 << 1 ),
    MAG_FORCED_MODE = ( 0b01 << 1 ),
    MAG_SLEEP_MODE  = ( 0b11 << 1 )
    } MAG_OP_MODE;

/**
 * @brief BMM150 factory trim coefficients, read out of NVM during mag_init()
 *        and used to compensate raw magnetometer readings.
 */
typedef struct _MAG_TRIM 
    {
    int8_t  dig_x1;
    int8_t  dig_y1;
    int8_t  dig_x2;
    int8_t  dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t  dig_xy1;
    int8_t   dig_xy2;
    uint16_t dig_xyz1;
    } MAG_TRIM;

/** @brief User magnetometer configuration settings, passed to mag_init */
typedef struct _MAG_CONFIG 
    {
    MAG_ODR_SETTING odr;            /* Magnetometer Output Data Rate      */
    MAG_OP_MODE     op_mode;        /* Magnetometer Operation Mode        */
    uint8_t         xy_repititions; /* Magnetometer XY Measurement Reps   */
    uint8_t         z_repititions;  /* Magnetometer Z  Measurement Reps   */
    } MAG_CONFIG;


/** @brief Standard status return codes for magnetometer driver operations */
typedef enum MAG_STATUS
    {
    MAG_OK              = 0,
    MAG_FAIL               ,
    MAG_UNSUPPORTED_OP     ,
    MAG_UNRECOGNIZED_OP    ,
    MAG_TIMEOUT            , 
    MAG_I2C_ERROR          ,
    MAG_ERROR              ,
    MAG_INIT_FAIL          ,
    MAG_CONFIG_FAIL        ,
    MAG_UNRECOGNIZED_ID    ,
    MAG_BUSY
    } MAG_STATUS;

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/** @brief Initialize the magnetometer                                       */
MAG_STATUS mag_init
    (
    MAG_CONFIG* mag_config_ptr
    );

/** @brief Getter for the static mag_data_ready flag                        */
bool mag_get_data_ready
    (
    void
    );

/** @brief Getter function for the magnetometer trim coefficients            */
MAG_TRIM mag_get_trim
    (
    void
    );

/**
 * @brief  Blocking read of magnetometer X/Y/Z.
 * @param[out] mag_raw: Destination struct for the magnetometer readout.
 * @return MAG_STATUS
 */
MAG_STATUS mag_get_xyz
    (
    MAG_RAW* mag_raw
    );

/**
  * @brief  Receicve magnetometer data in interrupt mode.
  * @return MAG_STATUS
  */
MAG_STATUS start_mag_read_IT
    (
    void
    );

/**
  * @brief  Handle a memory_rx interrupt signal from the magnetometer.
  * @return MAG_STATUS
  */
MAG_STATUS mag_it_handler
    (
    void
    );

#ifdef __cplusplus
}
#endif

#endif /* MAG_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
