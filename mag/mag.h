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

// might be able to move these into the implementation instead of condefs in the header
/** @brief Magnetometer (BMM150) I2C address, pre-shifted for HAL addressing  */
#define IMU_MAG_ADDR             0x10<<1

/** @brief BMM150 CHIP_ID expected value                                     */
#define MAG_ID                   0x32

/**
 * @brief Bitmasks/bitshifts used to unpack the BMM150's split 13/15-bit
 *        XY/Z/RHALL sample fields into contiguous 16-bit values.
 */
#define MAG_XY_LSB_BITMASK          0b11111000
#define MAG_XY_LSB_BITSHIFT         3 /* Bit 3 to position 0 */
#define MAG_XY_MSB_BITSHIFT         5 /* Bit 0 to position 5 */
#define MAG_Z_LSB_BITMASK           0b11111110
#define MAG_Z_LSB_BITSHIFT          1 /* Bit 1 to position 0 */
#define MAG_Z_MSB_BITSHIFT          7 /* Bit 0 to position 7 */
#define MAG_RHALL_LSB_BITMASK       0b11111100
#define MAG_RHALL_LSB_BITSHIFT      2 /* Bit 2 to position 0 */
#define MAG_RHALL_MSB_BITSHIFT      6 /* Bit 0 to position 6 */


/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/

#ifdef A0002_REV2 /* BMM150 */
    #define MAG_REG_CHIP_ID             0x40
    #define MAG_REG_DATAX_L             0x42
    #define MAG_REG_DATAX_H             0x43
    #define MAG_REG_DATAY_L             0x44
    #define MAG_REG_DATAY_H             0x45
    #define MAG_REG_DATAZ_L             0x46
    #define MAG_REG_DATAZ_H             0x47
    #define MAG_REG_HALLR_L             0x48
    #define MAG_REG_HALLR_H             0x49
    #define MAG_REG_INT                 0x4A
    #define MAG_REG_PWR_CTRL            0x4B
    #define MAG_REG_CTRL1               0x4C
    #define MAG_REG_CTRL2               0x4D
    #define MAG_REG_CTRL3               0x4E
    #define MAG_REG_LOW_THRESH          0x4F
    #define MAG_REG_HIGH_THRESH         0x50
    #define MAG_REG_REP_CTRL_XY         0x51
    #define MAG_REG_REP_CTRL_Z          0x52
    /* Trim Registers */
    #define MAG_TRIM_REG_X1             0x5D
    #define MAG_TRIM_REG_Y1             0x5E
    #define MAG_TRIM_REG_Z4_LSB         0x62
    #define MAG_TRIM_REG_Z4_MSB         0x63
    #define MAG_TRIM_REG_X2             0x64
    #define MAG_TRIM_REG_Y2             0x65
    #define MAG_TRIM_REG_Z2_LSB         0x68
    #define MAG_TRIM_REG_Z2_MSB         0x69
    #define MAG_TRIM_REG_Z1_LSB         0x6A
    #define MAG_TRIM_REG_Z1_MSB         0x6B
    #define MAG_TRIM_REG_XYZ1_LSB       0x6C
    #define MAG_TRIM_REG_XYZ1_MSB       0x6D
    #define MAG_TRIM_REG_Z3_LSB         0x6E
    #define MAG_TRIM_REG_Z3_MSB         0x6F
    #define MAG_TRIM_REG_XY2            0x70
    #define MAG_TRIM_REG_XY1            0x71
#endif

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

/** @brief Getter for the static mag_data_ready flag                        */
bool mag_get_mag_data_ready
    (
    void
    );

/** @brief Getter function for the magnetometer trim coefficients            */
MAG_TRIM mag_get_trim
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
