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

#if !defined( A0010 )
    #include "imu_legacy.h"
#endif

typedef struct {
    float accel[3];         /* g                                   */
    float gyro[3];          /* dps                                 */
    float quaternion[4];    /* [w, x, y, z], unit quat              */
    bool  quaternion_valid; /* false on legacy until AHRS warms up  */
    bool  is_valid;
} IMU_FlightData;

typedef enum {
    IMU_SYS_OK = 0,
    IMU_SYS_FAIL,
    IMU_SYS_BUSY,      /* async path: no new data yet */
    IMU_SYS_NO_DATA,
    IMU_SYS_INIT_FAIL
} IMU_SYS_STATUS;

IMU_SYS_STATUS imu_system_init   ( void );
IMU_SYS_STATUS imu_system_update ( IMU_FlightData* out ); /* non-blocking where backend allows */

#endif /* IMU_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
