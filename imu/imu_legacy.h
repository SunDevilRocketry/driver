/**
  ******************************************************************************
  * @file           : imu_legacy.h
  * @brief          : BMI270/BMM150 IMU Driver Interface (A0002 Rev2)
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Hardware Assumptions:
  *        Target: A0002 Rev2 PCB
  *        IMU:    Bosch BMI270 (6-axis accel + gyro), I2C address IMU_ADDR
  *        Mag:    Bosch BMM150 (magnetometer, auxiliary I2C passthrough),
  *                I2C address IMU_MAG_ADDR
  *        Bus:    I2C2
  *
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause (https://opensource.org/license/bsd-3-clause).
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_LEGACY_H
#define IMU_LEGACY_H

#if defined( A0010 )
#error "imu_legacy.h is not supported on the A0010 hardware platform"
#endif

#if !defined( A0002_REV2 )
#error "imu_legacy.h is only supported on the A0002_REV2 hardware platform"
#endif

#include <stdbool.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/** @brief IMU (BMI270) I2C address, pre-shifted for HAL 8-bit addressing     */
#define IMU_ADDR                0x68<<1

/** @brief Magnetometer (BMM150) I2C address, pre-shifted for HAL addressing  */
#define IMU_MAG_ADDR             0x10<<1

/** @brief BMI270 CHIP_ID expected value                                     */
#define IMU_ID                   0x24

/** @brief BMM150 CHIP_ID expected value                                     */
#define MAG_ID                   0x32

/* SDEC Subcommand Codes */
#define IMU_DUMP_CODE               0x01
#define IMU_POLL_CODE               0x02
#define IMU_LIST_CODE               0x03

/** @brief Blocking I2C HAL timeout (ms)                                     */
#define HAL_IMU_TIMEOUT             10

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

/* BMI270 Registers */
#define IMU_REG_CHIP_ID             0x00
#define IMU_REG_ERR_REG             0x02
#define IMU_REG_STATUS              0x03
#define IMU_REG_DATA_0              0x04    /* AUX_X (LSB) */
#define IMU_REG_DATA_1              0x05    /* AUX_X (MSB) */
#define IMU_REG_DATA_2              0x06    /* AUX_Y (LSB) */
#define IMU_REG_DATA_3              0x07    /* AUX_Y (MSB) */
#define IMU_REG_DATA_4              0x08    /* AUX_Z (LSB) */
#define IMU_REG_DATA_5              0x09    /* AUX_Z (MSB) */
#define IMU_REG_DATA_6              0x0A    /* AUX_R (LSB) */
#define IMU_REG_DATA_7              0x0B    /* AUX_R (MSB) */
#define IMU_REG_DATA_8              0x0C    /* ACC_X (LSB) */
#define IMU_REG_DATA_9              0x0D    /* ACC_X (MSB) */
#define IMU_REG_DATA_10             0x0E    /* ACC_Y (LSB) */
#define IMU_REG_DATA_11             0x0F    /* ACC_Y (MSB) */
#define IMU_REG_DATA_12             0x10    /* ACC_Z (LSB) */
#define IMU_REG_DATA_13             0x11    /* ACC_Z (MSB) */
#define IMU_REG_DATA_14             0x12    /* GYR_X (LSB) */
#define IMU_REG_DATA_15             0x13    /* GYR_X (MSB) */
#define IMU_REG_DATA_16             0x14    /* GYR_Y (LSB) */
#define IMU_REG_DATA_17             0x15    /* GYR_Y (MSB) */
#define IMU_REG_DATA_18             0x16    /* GYR_Z (LSB) */
#define IMU_REG_DATA_19             0x17    /* GYR_Z (MSB) */
#define IMU_REG_SENSORTIME_0        0x18
#define IMU_REG_SENSORTIME_1        0x19
#define IMU_REG_SENSORTIME_2        0x1A
#define IMU_REG_EVENT               0x1B
#define IMU_REG_INT_STATUS_0        0x1C
#define IMU_REG_INT_STATUS_1        0x1D
#define IMU_REG_SC_OUT_0            0x1E
#define IMU_REG_SC_OUT_1            0x1F
#define IMU_REG_WR_GEST_ACT         0x20
#define IMU_REG_INTERNAL_STATUS     0x21
#define IMU_REG_TEMPERATURE_0       0x22
#define IMU_REG_TEMPERATURE_1       0x23
#define IMU_REG_FIFO_LENGTH_0       0x24
#define IMU_REG_FIFO_LENGTH_1       0x25
#define IMU_REG_FIFO_DATA           0x26
#define IMU_REG_FEAT_PAGE           0x2F
#define IMU_REG_FEATURES            0x30
#define IMU_REG_ACC_CONF            0x40
#define IMU_REG_ACC_RANGE           0x41
#define IMU_REG_GYR_CONF            0x42
#define IMU_REG_GYR_RANGE           0x43
#define IMU_REG_AUX_CONF            0x44
#define IMU_REG_FIFO_DOWNS          0x45
#define IMU_REG_FIFO_WTM_0          0x46
#define IMU_REG_FIFO_WTM_1          0x47
#define IMU_REG_FIFO_CONFIG_0       0x48
#define IMU_REG_FIFO_CONFIG_1       0x49
#define IMU_REG_SATURATION          0x4A
#define IMU_REG_AUX_DEV_ID          0x4B
#define IMU_REG_AUX_IF_CONF         0x4C
#define IMU_REG_AUX_RD_ADDR         0x4D
#define IMU_REG_AUX_WR_ADDR         0x4E
#define IMU_REG_AUX_WR_DATA         0x4F
#define IMU_REG_ERR_REG_MSK         0x52
#define IMU_REG_INT1_IO_CTRL        0x53 
#define IMU_REG_INT2_IO_CTRL        0x54
#define IMU_REG_INT_LATCH           0x55
#define IMU_REG_INT1_MAP_FEAT       0x56
#define IMU_REG_INT2_MAP_FEAT       0x57
#define IMU_REG_INT_MAP_DATA        0x58
#define IMU_REG_INIT_CTRL           0x59
#define IMU_REG_INIT_ADDR_0         0x5B
#define IMU_REG_INIT_ADDR_1         0x5C
#define IMU_REG_INIT_DATA           0x5E
#define IMU_REG_INTERNAL_ERROR      0x5F
#define IMU_REG_AUX_IF_TRIM         0x68
#define IMU_REG_GYR_CRT_CONF        0x69
#define IMU_REG_NVM_CONF            0x6A
#define IMU_REG_IF_CONF             0x6B
#define IMU_REG_DRV                 0x6C
#define IMU_REG_ACC_SELF_TEST       0x6D
#define IMU_REG_GYR_SELF_TEST_AXES  0x6E
#define IMU_REG_NV_CONF             0x70
#define IMU_REG_OFFSET_0            0x71
#define IMU_REG_OFFSET_1            0x72
#define IMU_REG_OFFSET_2            0x73
#define IMU_REG_OFFSET_3            0x74
#define IMU_REG_OFFSET_4            0x75
#define IMU_REG_OFFSET_5            0x76
#define IMU_REG_OFFSET_6            0x77
#define IMU_REG_PWR_CONF            0x7C
#define IMU_REG_PWR_CTRL            0x7D
#define IMU_REG_CMD                 0x7E

/* BMM150 Registers */
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


/*------------------------------------------------------------------------------
 Typedefs 
------------------------------------------------------------------------------*/

/** @brief Raw accel/gyro/mag counts as read off the IMU and magnetometer     */
typedef struct _IMU_RAW {
    int16_t    accel_x;
    int16_t    accel_y;
    int16_t    accel_z;
    int16_t    gyro_x ;
    int16_t    gyro_y ;
    int16_t    gyro_z ;
    int16_t    mag_x;
    int16_t    mag_y;
    int16_t    mag_z;
    uint16_t    mag_hall;
} IMU_RAW;

/** @brief Processed/estimated vehicle attitude, rates, and kinematic state   */
typedef struct _STATE_ESTIMATION {
	float roll_angle;
	float pitch_angle;
    float yaw_angle;
	float roll_rate;
	float pitch_rate;
    float yaw_rate;
    float velocity;
    float velo_x;
    float velo_y;
    float velo_z;     
	float position;
} STATE_ESTIMATION;

/** @brief Physical-unit converted accel/gyro/mag data                       */
typedef struct _IMU_CONVERTED {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x ;
    float gyro_y ;
    float gyro_z ;
    float mag_x ;
    float mag_y ;
    float mag_z ;
} IMU_CONVERTED;

/** @brief Aggregate struct containing converted IMU data and state estimate */
typedef struct _IMU_DATA 
	{
    IMU_CONVERTED imu_converted;
    STATE_ESTIMATION state_estimate;
	} IMU_DATA;

/** @brief Per-axis accel/gyro zero-offset calibration values                */
typedef struct _IMU_OFFSET {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x ;
    float gyro_y ;
    float gyro_z ;
} IMU_OFFSET;

/** @brief Sensor enable bitfield, written to IMU_REG_PWR_CTRL                */
typedef enum _IMU_SENSOR_ENABLE
    {
    IMU_DISABLE_SENSORS      = 0b00000000,
    IMU_ENABLE_AUX           = 0b00000001,
    IMU_ENABLE_GYRO          = 0b00000010, 
    IMU_ENABLE_ACC           = 0b00000100,
    IMU_ENABLE_TEMP          = 0b00001000,
    IMU_ENABLE_AUX_GYRO      = 0b00000011,
    IMU_ENABLE_AUX_ACC       = 0b00000101,
    IMU_ENABLE_AUX_GYRO_ACC  = 0b00000111,
    IMU_ENABLE_GYRO_ACC_TEMP = 0b00001110,
    IMU_ENABLE_ALL           = 0b00001111
    } IMU_SENSOR_ENABLE;

/** @brief Accel/gyro output data rate, written to IMU_REG_ACC_CONF/GYR_CONF  */
typedef enum _IMU_ODR_SETTING
    {
    IMU_ODR_0P78 = 1, /* 25/32 Hz */ 
    IMU_ODR_1P5     , /* 25/16 Hz */
    IMU_ODR_3P1     , /* 25/8  Hz */
    IMU_ODR_6P25    , /* 25/4  Hz */
    IMU_ODR_12P5    , /* 25/2  Hz */
    IMU_ODR_25      , /* 25    Hz */
    IMU_ODR_50      , /* 50    Hz */
    IMU_ODR_100     , /* 100   Hz */
    IMU_ODR_200     , /* 200   Hz */
    IMU_ODR_400     , /* 400   Hz */
    IMU_ODR_800     , /* 800   Hz */
    IMU_ODR_1K6     , /* 1.6  kHz */
    IMU_ODR_3K2
    } IMU_ODR_SETTING;

/** @brief Accel/gyro digital filter configuration                           */
typedef enum _IMU_FILTER_CONFIG
    {
    IMU_FILTER_OSR4_AVG1 = 0            , /* OSR4 Filter, No Average       */
    IMU_FILTER_OSR2_AVG2 = ( 0x01 << 4 ), /* OSR2 Filter, 2 Sample Average */
    IMU_FILTER_NORM_AVG4 = ( 0x02 << 4 ), /* Normal Mode, 4 Sample Average */
    IMU_FILTER_CIC_AVG8  = ( 0x03 << 4 ), /* CIC Filter , 8 Sample Average */
    IMU_FILTER_AVG16     = ( 0x04 << 4 ), /* 16 Sample Average             */
    IMU_FILTER_AVG32     = ( 0x05 << 4 ), /* 32 Sample Average             */
    IMU_FILTER_AVG64     = ( 0x06 << 4 ), /* 64 Sample Average             */
    IMU_FILTER_AVG128    = ( 0x07 << 4 )  /* 128 Sample Average            */
    } IMU_FILTER_CONFIG;

/** @brief Selects filter vs. averaging mode for the accel/gyro filter chain */
typedef enum _IMU_FILTER_MODE
    {
    IMU_FILTER_FILTER_MODE  = ( 0x01 << 7 ),
    IMU_FILTER_AVERAGE_MODE = ( 0x00 << 7 )
    } IMU_FILTER_MODE;

/** @brief Accelerometer full-scale measurement range                        */
typedef enum _IMU_ACC_RANGE
    {
    IMU_ACC_RANGE_2G = 0, /* +- 2g  */ 
    IMU_ACC_RANGE_4G    , /* +- 4g  */
    IMU_ACC_RANGE_8G    , /* +- 8g  */
    IMU_ACC_RANGE_16G     /* +- 16g */
    } IMU_ACC_RANGE;

/** @brief Gyroscope full-scale measurement range                            */
typedef enum _IMU_GYRO_RANGE
    {
    IMU_GYRO_RANGE_2000 = 0, /* +- 2000 deg/s */
    IMU_GYRO_RANGE_1000    , /* +- 1000 deg/s */
    IMU_GYRO_RANGE_500     , /* +- 500  deg/s */
    IMU_GYRO_RANGE_250     , /* +- 250  deg/s */
    IMU_GYRO_RANGE_125       /* +- 125  deg/s */
    } IMU_GYRO_RANGE;

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

/** @brief User IMU configuration settings, passed in to imu_init()          */
typedef struct _IMU_CONFIG 
	{
    IMU_SENSOR_ENABLE sensor_enable;      /* Enabled Sensors                    */
    IMU_ODR_SETTING   acc_odr;            /* Accelerometer Output Data Rate     */ 
    IMU_ODR_SETTING   gyro_odr;           /* Gyroscope Output Data Rate         */
    MAG_ODR_SETTING   mag_odr;            /* Magnetometer Output Data Rate      */
    IMU_FILTER_CONFIG acc_filter;         /* Accelerometer Filter Config        */
    IMU_FILTER_CONFIG gyro_filter;        /* Gyroscope Filter Config            */
    IMU_FILTER_MODE   acc_filter_mode;    /* Accelerometer Filtering Mode       */
    IMU_FILTER_MODE   gyro_filter_mode;   /* Gyroscope Filtering Mode           */
    IMU_ACC_RANGE     acc_range;          /* Accelerometer Measurement Range    */
    IMU_GYRO_RANGE    gyro_range;         /* Gyroscope Measurement Range        */
    MAG_OP_MODE       mag_op_mode;        /* Magnetometer Operation Mode        */
    uint8_t           mag_xy_repititions; /* Magnetometer XY Measurement Reps   */
    uint8_t           mag_z_repititions;  /* Magnetometer Z  Measurement Reps   */
	} IMU_CONFIG;

/** @brief Standard status return codes for all IMU driver operations        */
typedef enum IMU_STATUS
	{
    IMU_OK              = 0,
    IMU_FAIL               ,
    IMU_UNSUPPORTED_OP     ,
    IMU_UNRECOGNIZED_OP    ,
    IMU_TIMEOUT            , 
    IMU_I2C_ERROR          ,
    IMU_MAG_ERROR          ,
    IMU_ERROR              ,
    IMU_INIT_FAIL          ,
    IMU_CONFIG_FAIL        ,
    IMU_MAG_UNRECOGNIZED_ID,
    IMU_MAG_INIT_FAIL      ,
    IMU_BUSY
	} IMU_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/** @brief Initialize the IMU (BMI270 config load + BMM150 mag_init)         */
IMU_STATUS imu_init
    (
    IMU_CONFIG* imu_config_ptr /* IMU Configuration Settings */
    );

/**
 * @brief  Blocking read of accelerometer X/Y/Z.
 * @param  pIMU: Destination struct for the accel readout.
 */
IMU_STATUS imu_get_accel_xyz
    (
    IMU_RAW *pIMU
    );

/**
 * @brief  Blocking read of gyroscope X/Y/Z.
 * @param  pIMU: Destination struct for the gyro readout.
 */
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_RAW *pIMU
    );

/**
 * @brief  Blocking read of accelerometer and gyroscope X/Y/Z in one burst.
 * @param  pIMU: Destination struct for the accel + gyro readout.
 */
IMU_STATUS imu_get_accel_and_gyro
    (
    IMU_RAW *pIMU
    );

/**
 * @brief  Blocking read of magnetometer X/Y/Z.
 * @param  pIMU: Destination struct for the magnetometer readout.
 */
IMU_STATUS imu_get_mag_xyz
    (
    IMU_RAW *pIMU
    );

/**
 * @brief  Reads and verifies the IMU's CHIP_ID register.
 * @param  pdevice_id: Destination for the returned device ID.
 */
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    );

/* Change configuration of accel, gyro, mag */
void IMU_config
    (
    IMU_CONFIG *pimu_config,
    uint8_t accel_setting,
    uint16_t gyro_setting,
    uint16_t mag_setting
    );

/** @brief Getter for the static imu_data_ready flag                         */
bool imu_get_imu_data_ready
    (
    void
    );

/** @brief Getter for the static mag_data_ready flag                        */
bool imu_get_mag_data_ready
    (
    void
    );

/** @brief Triggers an interrupt-mode (non-blocking) accel + gyro read       */
IMU_STATUS start_imu_read_IT(void);

/** @brief Interrupt handler - call on I2C memory-rx complete interrupt      */
IMU_STATUS imu_it_handler();

/**
 * @brief  Consumer - retrieves the latest interrupt-mode accel/gyro/mag data.
 * @param  cpy_ptr: Destination struct for the copied sensor readout.
 */
IMU_STATUS get_imu_it(IMU_RAW* cpy_ptr);

/** @brief Getter function for the magnetometer trim coefficients            */
MAG_TRIM imu_get_mag_trim();

#ifdef __cplusplus
}
#endif

#endif /* IMU_LEGACY_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
