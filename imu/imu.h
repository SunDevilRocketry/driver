/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : Unified IMU System Interface
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Abstracts the underlying differences between legacy 9-axis I2C 
  *        architectures and newer 6-axis SPI DMA architectures.
  *
  *        This interface serves as a static contract across all projects. 
  *        The implementation is swappable: `imu_dflt.c` provides the 
  *        working default, but individual projects can supply a custom 
  *        source file to manage specific power-mode and full-scale scheduling.
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms found in the LICENSE file in the root
  * directory of this component. If absent, BSD-3-Clause applies:
  * https://opensource.org/license/bsd-3-clause
  ******************************************************************************
  */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "math_sdr.h"

#if !defined( A0010 )
    #include "imu_legacy.h"
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* I2C Address */
#define IMU_ADDR                0x68<<1

/* Device ID */
#if   defined( A0002_REV1 )
    #define IMU_ID                  0x71
#elif defined( A0002_REV2 )
    #define IMU_ID                  0x24
#endif

/* SDEC Subcommand Codes */
#define IMU_DUMP_CODE               0x01
#define IMU_POLL_CODE               0x02
#define IMU_LIST_CODE               0x03

/* Timeouts */
#define HAL_IMU_TIMEOUT             10


/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
#if defined( A0002_REV1 )
    /* MPU 9250 Registers */
    #define IMU_REG_GYRO_CONFIG         0x1B
    #define IMU_REG_ACCEL_CONFIG        0x1C
    #define IMU_REG_ACCEL_XOUT_H        0x3B
    #define IMU_REG_ACCEL_XOUT_L        0x3C
    #define IMU_REG_ACCEL_YOUT_H        0X3D
    #define IMU_REG_ACCEL_YOUT_L        0x3E
    #define IMU_REG_ACCEL_ZOUT_H        0x3F
    #define IMU_REG_ACCEL_ZOUT_L        0x40
    #define IMU_REG_TEMP_OUT_H          0x41
    #define IMU_REG_TEMP_OUT_L          0x42
    #define IMU_REG_GYRO_XOUT_H         0x43
    #define IMU_REG_GYRO_XOUT_L         0x44
    #define IMU_REG_GYRO_YOUT_H         0X45
    #define IMU_REG_GYRO_YOUT_L         0x46
    #define IMU_REG_GYRO_ZOUT_H         0x47
    #define IMU_REG_GYRO_ZOUT_L         0x48
    #define IMU_REG_MAG_XOUT_H          0x04
    #define IMU_REG_MAG_XOUT_L          0x03
    #define IMU_REG_MAG_YOUT_H          0X06
    #define IMU_REG_MAG_YOUT_L          0x05
    #define IMU_REG_MAG_ZOUT_H          0x08
    #define IMU_REG_MAG_ZOUT_L          0x07
    #define IMU_REG_WHO_AM_I            0x75
#elif defined( A0002_REV2  )
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
#endif

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

typedef struct 
    {
    float accel[3];         /* m/s^2 (SI)                           */
    float gyro[3];          /* dps                                  */
    float quaternion[4];    /* [w, x, y, z], unit quat              */
    bool  quaternion_valid; /* false on legacy until AHRS warms up  */
    bool  is_valid;
    } IMU_FlightData;

typedef enum 
    {
    IMU_SYS_OK = 0,
    IMU_SYS_FAIL,
    IMU_SYS_BUSY,      /* async path: no new data yet */
    IMU_SYS_NO_DATA,
    IMU_SYS_INIT_FAIL
    } IMU_SYS_STATUS;

/**
 * @return IMU_SYS_STATUS
 */
IMU_SYS_STATUS imu_system_init
    (
    void
    );

/**
 * @param  out: Destination for unified flight data.
 * @return IMU_SYS_STATUS
 */
IMU_SYS_STATUS imu_system_update
    (
    IMU_FlightData* out
    ); /* non-blocking where backend allows */

#endif /* IMU_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
