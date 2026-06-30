/**
  ******************************************************************************
  * @file           : imu_dflt.c
  * @brief          : Default Unified IMU System Translation Implementation
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Default implementation of the imu.h contract. This file serves as 
  *        an reference; projects requiring custom configurations—
  *        such as mid-flight state transitions or alternate ODRs—should exclude 
  *        this file from their build and supply a project-specific translation layer.
  *
  *        Unit Translation: Converts backend-specific native units (such as 
  *        the SI units reported by imu_lsm.c) into the unified contract units 
  *        of IMU_FlightData (accelerometer in [g], gyroscope in [dps]).
  *
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms found in the LICENSE file in the root
  * directory of this component. If absent, BSD-3-Clause applies:
  * https://opensource.org/license/bsd-3-clause
  ******************************************************************************
  */

#include "imu.h"

#if defined( A0010 )

    #include "imu_lsm.h"

    #define IMU_STANDARD_GRAVITY    ( 9.80665f )
    #define IMU_RAD_TO_DEG          ( 57.29577951f )

    IMU_SYS_STATUS imu_system_init(void)
    {
        IMU_CONFIG config = {
            .odr         = IMU_ODR_1920HZ,
            .acc_fs      = IMU_ACC_FS_16G,
            .gyro_fs     = IMU_GYRO_FS_2000DPS,
            .acc_mode    = IMU_ACC_MODE_HP,
            .gyro_mode   = IMU_GYRO_MODE_HP,
            .sflp_enable = true,
            .fifo_enable = true
        };

        if (imu_init(&config) != IMU_OK) {
            return IMU_SYS_INIT_FAIL;
        }
        return IMU_SYS_OK;
    }

    IMU_SYS_STATUS imu_system_update(IMU_FlightData* out)
    {
        if (!imu_has_new_data()) {
            return IMU_SYS_BUSY;
        }

        IMU_RAW raw;
        IMU_SFLP_DATA sflp;
        
        if (imu_get_latest(&raw, &sflp) != IMU_OK) {
            return IMU_SYS_FAIL;
        }

        IMU_DATA physical;
        imu_scale_raw(&raw, &physical);

        /* imu_lsm.c reports SI (m/s^2, rad/s) internally; convert to the
           unified contract's [g] / [dps] here. */
        out->accel[0] = physical.accel_x / IMU_STANDARD_GRAVITY;
        out->accel[1] = physical.accel_y / IMU_STANDARD_GRAVITY;
        out->accel[2] = physical.accel_z / IMU_STANDARD_GRAVITY;

        out->gyro[0]  = physical.gyro_x * IMU_RAD_TO_DEG;
        out->gyro[1]  = physical.gyro_y * IMU_RAD_TO_DEG;
        out->gyro[2]  = physical.gyro_z * IMU_RAD_TO_DEG;

        out->quaternion[0] = sflp.quat_w;
        out->quaternion[1] = sflp.quat_x;
        out->quaternion[2] = sflp.quat_y;
        out->quaternion[3] = sflp.quat_z;

        out->quaternion_valid = true;
        out->is_valid         = true;

        return IMU_SYS_OK;
    }

    #undef IMU_STANDARD_GRAVITY
    #undef IMU_RAD_TO_DEG

   /* -------------------------------------------------------------------
       Example DMA Completion / Error Callbacks (Reference Only)
      -------------------------------------------------------------------
       The IMU driver's non-blocking async path expects routing from 
       project's global HAL SPI callbacks to the driver's ISR hooks. 
   
       To avoid linker collisions with other peripherals sharing the SPI 
       bus, do not define these globally in the driver. Instead, integrate 
       them into your project's existing callbacks as shown below:

         void HAL_SPI_TxRxCpltCallback( SPI_HandleTypeDef* hspi )
         {
             if ( hspi == &IMU_SPI )
            {
                 imu_process_async_cb();
             }
         }

         void HAL_SPI_ErrorCallback( SPI_HandleTypeDef* hspi )
         {
             if ( hspi == &IMU_SPI )
             {
                 imu_process_async_error_cb();
             }
         }
    */

#else /* Legacy Target */

    #include "imu_legacy.h"

    #define LEGACY_ACCEL_SCALE   (16.0f / 32768.0f)
    #define LEGACY_GYRO_SCALE    (2000.0f / 32768.0f)

    IMU_SYS_STATUS imu_system_init(void)
    {
        IMU_CONFIG config = {
            .sensor_enable    = IMU_ENABLE_GYRO_ACC_TEMP,
            .acc_odr          = IMU_ODR_100,
            .gyro_odr         = IMU_ODR_100,
            .acc_filter       = IMU_FILTER_NORM_AVG4,
            .gyro_filter      = IMU_FILTER_NORM_AVG4,
            .acc_filter_mode  = IMU_FILTER_FILTER_MODE,
            .gyro_filter_mode = IMU_FILTER_FILTER_MODE,
            .acc_range        = IMU_ACC_RANGE_16G,
            .gyro_range       = IMU_GYRO_RANGE_2000,
            .mag_op_mode      = MAG_SLEEP_MODE
        };

        if (imu_init(&config) != IMU_OK) {
            return IMU_SYS_INIT_FAIL;
        }
        return IMU_SYS_OK;
    }

    IMU_SYS_STATUS imu_system_update(IMU_FlightData* out)
    {
        IMU_RAW raw;
        IMU_STATUS status;

        #ifdef A0002_REV2
            status = imu_get_accel_and_gyro(&raw);
        #else
            status = imu_get_accel_xyz(&raw);
            if (status == IMU_OK) {
                status = imu_get_gyro_xyz(&raw);
            }
        #endif

        if (status != IMU_OK) {
            return IMU_SYS_FAIL;
        }

        out->accel[0] = (float)raw.accel_x * LEGACY_ACCEL_SCALE;
        out->accel[1] = (float)raw.accel_y * LEGACY_ACCEL_SCALE;
        out->accel[2] = (float)raw.accel_z * LEGACY_ACCEL_SCALE;

        out->gyro[0]  = (float)raw.gyro_x * LEGACY_GYRO_SCALE;
        out->gyro[1]  = (float)raw.gyro_y * LEGACY_GYRO_SCALE;
        out->gyro[2]  = (float)raw.gyro_z * LEGACY_GYRO_SCALE;

        out->quaternion[0] = 0.0f;
        out->quaternion[1] = 0.0f;
        out->quaternion[2] = 0.0f;
        out->quaternion[3] = 0.0f;
        
        out->quaternion_valid = false;
        out->is_valid         = true;

        return IMU_SYS_OK;
    }

#endif

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
