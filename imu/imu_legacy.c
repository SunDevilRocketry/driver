/**
  ******************************************************************************
  * @file           : imu_legacy.c
  * @brief          : BMI270/BMM150 IMU Driver Implementation (A0002 Rev2)
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Two data-retrieval paths are provided:
  *          - Blocking: imu_get_accel_xyz() / imu_get_gyro_xyz() /
  *            imu_get_accel_and_gyro() / imu_get_mag_xyz() poll the I2C bus
  *            directly and return once the transfer completes.
  *          - Interrupt-mode: start_imu_read_IT() kicks off a non-blocking
  *            I2C read; imu_it_handler() (called from the I2C memory-rx ISR)
  *            chains the accel/gyro read into a magnetometer read and marks
  *            the result ready; get_imu_it() is the non-blocking consumer.
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

#if defined( A0002_REV1 )
    #error "A0002_REV1 support has been removed."
#endif


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdatomic.h>

/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "imu_legacy.h"

#include <math.h>

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/** @brief BMI270 configuration, loaded into the device during imu_init() */
uint8_t bmi270_init_file[] = {
    #include "bmi270_init_file.tbin"
};

/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/

/** @brief Raw double-buffer target for interrupt-mode accel/gyro reads       */
uint8_t imu_raw_buffer[12];
/** @brief Parsed interrupt-mode accel/gyro/mag sample, published by imu_it_handler() */
IMU_RAW imu_raw_processed;
/** @brief Set true by imu_it_handler() once imu_raw_processed's accel/gyro fields are valid */
static atomic_bool imu_data_ready;

/** @brief Raw double-buffer target for interrupt-mode magnetometer reads     */
uint8_t mag_raw_buffer[8];
/** @brief Set true by imu_it_handler() once imu_raw_processed's mag fields are valid */
static atomic_bool mag_data_ready;

/** @brief BMM150 factory trim coefficients, populated by mag_init()          */
MAG_TRIM mag_trim;

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/** @brief Initialize the magnetometer                                       */
static IMU_STATUS mag_init
    (
    IMU_CONFIG* imu_config_ptr
    );

/** @brief Read IMU registers (blocking)                                     */
static IMU_STATUS read_imu_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/** @brief Write to a specified IMU register (blocking)                      */
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    );

/** @brief Write IMU registers (blocking)                                    */
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint32_t num_regs  /* Number of registers */
    ); 

/** @brief Read Magnetometer registers (blocking)                            */
static IMU_STATUS read_mag_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/** @brief Write to a specified magnetometer register (blocking)             */
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ); 

/** @brief Read IMU registers (interrupt mode)                               */
static IMU_STATUS read_imu_regs_IT
    (
    uint8_t  reg_addr, /* Register address            */
    uint8_t* data_ptr, /* Register data               */ 
    uint8_t  num_regs  /* Number of registers to read */
    );

/** @brief Read Magnetometer registers (interrupt mode)                      */
static IMU_STATUS read_mag_regs_IT 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/**
  * @brief  Initializes the IMU (BMI270 config load + sensor config + BMM150 mag_init).
  * @param  imu_config_ptr: Pointer to user configuration struct.
  * @return IMU_STATUS
  */
IMU_STATUS imu_init 
    (
    IMU_CONFIG* imu_config_ptr /* IMU Configuration */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS imu_status;          /* IMU API call return codes       */
uint8_t    imu_dev_id;          /* IMU identification code         */
uint8_t    imu_status_reg;      /* Contents of IMU status register */
uint8_t    imu_acc_conf;        /* IMU ACC_CONF Register contents  */
uint8_t    imu_gyr_conf;        /* IMU GYR_CONF Register contents  */
uint8_t    imu_sensor_data[12]; /* IMU Sensor Data                 */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
imu_status     = IMU_OK;
imu_dev_id     = 0;
imu_status_reg = 0;
imu_acc_conf   = ( imu_config_ptr -> acc_odr         ) |
                 ( imu_config_ptr -> acc_filter      ) |
                 ( imu_config_ptr -> acc_filter_mode );
imu_gyr_conf   = ( imu_config_ptr -> gyro_odr        ) |
                 ( imu_config_ptr -> gyro_filter     ) |
                 ( 1 << 7 );
memset( &imu_sensor_data[0], 0, sizeof( imu_sensor_data ) );

/* clear double buffer if using IMU with IT */
memset( &imu_raw_buffer, 0, sizeof( imu_raw_buffer ) );
imu_data_ready = false;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* disable interrupts while initializing */
HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);

/* Read IMU ID and verify correct ID */
imu_status = imu_get_device_id( &imu_dev_id );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* BMI270 Initialization Sequence */
    /* Disable advanced power save */
    imu_status = write_imu_reg( IMU_REG_PWR_CONF, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    HAL_Delay( 1 );

    /* Prepare Config Load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x00 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }
    
    /* Load the initialization data */
    imu_status = write_imu_regs( IMU_REG_INIT_DATA   , 
                                 &bmi270_init_file[0], 
                                 sizeof( bmi270_init_file ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Complete config load */
    imu_status = write_imu_reg( IMU_REG_INIT_CTRL, 0x01 );
    if ( imu_status != IMU_OK )
        {
        return IMU_INIT_FAIL;
        }

    /* Check if Initialization was Successful */
    HAL_Delay( 150 );
    imu_status = read_imu_regs( IMU_REG_INTERNAL_STATUS, 
                                &imu_status_reg        ,
                                sizeof( imu_status_reg ) );
    if ( imu_status != IMU_OK || !( imu_status_reg & 0b00000001 ) )
        {
        return IMU_INIT_FAIL;
        }

/* Initial IMU Configuration */
    /* Enable Sensors */
    imu_status = write_imu_reg( IMU_REG_PWR_CTRL, 
                                imu_config_ptr -> sensor_enable );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Configure the Accelerometer */
    imu_status = write_imu_reg( IMU_REG_ACC_CONF, imu_acc_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = write_imu_reg( IMU_REG_ACC_RANGE,
                                imu_config_ptr -> acc_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    
    /* Configure the Gyroscope */
    imu_status = write_imu_reg( IMU_REG_GYR_CONF, imu_gyr_conf );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }
    imu_status = write_imu_reg( IMU_REG_GYR_RANGE, 
                                imu_config_ptr -> gyro_range );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Disable Advanced Power Save */
    imu_status = write_imu_reg( IMU_REG_PWR_CONF, 0x02 );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL;
        }

    /* Readout sensor registers */
    imu_status = read_imu_regs( IMU_REG_DATA_8     , 
                                &imu_sensor_data[0], 
                                sizeof( imu_sensor_data ) );
    if ( imu_status != IMU_OK )
        {
        return IMU_CONFIG_FAIL; 
        }

    /* Initialize the magnetometer */
    imu_status = mag_init( imu_config_ptr );
    if ( imu_status != IMU_OK )
        {
        return IMU_MAG_INIT_FAIL;
        }

/* re-enable interrupts while initializing */
HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

/* IMU Inititialization Successful */
return IMU_OK;
} /* imu_init */


/**
  * @brief  Blocking read of accelerometer X/Y/Z.
  * @param  pIMU: Destination struct for the accel readout (size: 12 bytes).
  * @return IMU_STATUS
  */
IMU_STATUS imu_get_accel_xyz
    (
    IMU_RAW *pIMU /* size: 12 bytes */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t       regAccel[6];    /* Bytes from accelerometer registers */
uint16_t      accel_x_raw ;   /* Raw sensor readouts                */    
uint16_t      accel_y_raw ;  
uint16_t      accel_z_raw ; 
IMU_STATUS    imu_status;     /* IMU status codes                   */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read ACCEL_X, ACCEL_Y, ACCEL_Z high byte and low byte registers */
imu_status = read_imu_regs( IMU_REG_DATA_8, 
                            &regAccel[0]  ,
                            sizeof( regAccel ) );

/* Check for HAL IMU error */
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Combine high byte and low byte to 16 bit data */ 
accel_x_raw    = ( (uint16_t) regAccel[1] ) << 8  | regAccel[0];
accel_y_raw    = ( (uint16_t) regAccel[3] ) << 8  | regAccel[2];
accel_z_raw    = ( (uint16_t) regAccel[5] ) << 8  | regAccel[4]; 

/* Export data to IMU sstruct */
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw;

return IMU_OK;
} /* imu_get_accel_xyz */


/**
  * @brief  Blocking read of gyroscope X/Y/Z.
  * @param  pIMU: Destination struct for the gyro readout.
  * @return IMU_STATUS
  */
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_RAW *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regGyro[6];   /* Bytes from gyro registers */
int16_t    gyro_x_raw;    /* Raw gyro sensor readouts  */
int16_t    gyro_y_raw; 
int16_t    gyro_z_raw; 
IMU_STATUS  imu_status;   /* IMU status return codes   */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read GYRO_X, GYRO_Y, GYRO_Z high byte and low byte registers */
imu_status = read_imu_regs( IMU_REG_DATA_14, 
                            &regGyro[0]    , 
                            sizeof( regGyro ) );
 
/* Check for HAL IMU error */
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Combine high byte and low byte to 16 bit data  */
gyro_x_raw = (int16_t) ( (uint16_t) regGyro[1] << 8 | regGyro[0] );
gyro_y_raw = (int16_t) ( (uint16_t) regGyro[3] << 8 | regGyro[2] );
gyro_z_raw = (int16_t) ( (uint16_t) regGyro[5] << 8 | regGyro[4] );

/* Export Sensor Readouts */
pIMU->gyro_x = gyro_x_raw;
pIMU->gyro_y = gyro_y_raw;
pIMU->gyro_z = gyro_z_raw; 

return IMU_OK;
} /* imu_get_gyro_xyz */


/**
  * @brief  Blocking read of accelerometer and gyroscope X/Y/Z in a single burst.
  * @param  pIMU: Destination struct for the accel + gyro readout.
  * @return IMU_STATUS
  */
IMU_STATUS imu_get_accel_and_gyro
    (
    IMU_RAW *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regRaw[12];  /* Bytes from raw registers */
int16_t    accel_x_raw;   /* Raw accel sensor readouts  */
int16_t    accel_y_raw; 
int16_t    accel_z_raw; 
int16_t    gyro_x_raw;    /* Raw gyro sensor readouts  */
int16_t    gyro_y_raw; 
int16_t    gyro_z_raw; 
IMU_STATUS  imu_status;   /* IMU status return codes   */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read ACCEL and GYRO high byte and low byte registers */
imu_status = read_imu_regs( IMU_REG_DATA_8, 
                                &regRaw[0]     , 
                                sizeof( regRaw ) );

 
/* Check for HAL IMU error */
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Combine high byte and low byte to 16 bit data  */
accel_x_raw = (int16_t) ( (uint16_t) regRaw[1] << 8 | regRaw[0] );
accel_y_raw = (int16_t) ( (uint16_t) regRaw[3] << 8 | regRaw[2] );
accel_z_raw = (int16_t) ( (uint16_t) regRaw[5] << 8 | regRaw[4] );
gyro_x_raw  = (int16_t) ( (uint16_t) regRaw[7] << 8 | regRaw[6] );
gyro_y_raw  = (int16_t) ( (uint16_t) regRaw[9] << 8 | regRaw[8] );
gyro_z_raw  = (int16_t) ( (uint16_t) regRaw[11] << 8 | regRaw[10] );

/* Export Sensor Readouts */
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw; 
pIMU->gyro_x = gyro_x_raw;
pIMU->gyro_y = gyro_y_raw;
pIMU->gyro_z = gyro_z_raw; 

return IMU_OK;
} /* imu_get_accel_and_gyro */


/**
  * @brief  Blocking read of magnetometer X/Y/Z.
  * @param  pIMU: Destination struct for the magnetometer readout.
  * @return IMU_STATUS
  */
IMU_STATUS imu_get_mag_xyz
    (
    IMU_RAW *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t    regMag[8];      /* Magnetometer register bytes      */
int16_t    mag_x_raw;      /* Raw magnetometer sensor readouts */
int16_t    mag_y_raw;
int16_t    mag_z_raw;
uint16_t   mag_hall_raw;
IMU_STATUS imu_status;     /* IMU status return codes           */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers */
imu_status = read_mag_regs( MAG_REG_DATAX_L, 
                            &regMag[0]     , 
                            sizeof( regMag ) );

/* Check for HAL IMU error */
if ( imu_status == IMU_TIMEOUT )
    {
    return IMU_TIMEOUT;
    }

/* Combine high byte and low byte to 16 bit data */
mag_x_raw  = (   (uint16_t) regMag[1]                        << MAG_XY_MSB_BITSHIFT ) | 
             ( ( (uint16_t) regMag[0] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
mag_y_raw  = (   (uint16_t) regMag[3]                        << MAG_XY_MSB_BITSHIFT ) | 
             ( ( (uint16_t) regMag[2] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
mag_z_raw  = (   (uint16_t) regMag[5]                        << MAG_Z_MSB_BITSHIFT  ) | 
             ( ( (uint16_t) regMag[4] & MAG_Z_LSB_BITMASK )  >> MAG_Z_LSB_BITSHIFT  );
mag_hall_raw = (   (uint16_t) regMag[7]                            << MAG_RHALL_MSB_BITSHIFT  ) | 
               ( ( (uint16_t) regMag[6] & MAG_RHALL_LSB_BITMASK )  >> MAG_RHALL_LSB_BITSHIFT  );

/* Export sensor data */
pIMU->mag_x    = mag_x_raw;
pIMU->mag_y    = mag_y_raw;
pIMU->mag_z    = mag_z_raw;
pIMU->mag_hall = mag_hall_raw;

return IMU_OK;
} /* imu_get_mag_xyz */


/**
  * @brief  Reads and verifies the IMU's CHIP_ID register.
  * @param  pdevice_id: Destination for the returned device ID.
  * @return IMU_STATUS: IMU_UNRECOGNIZED_OP if the ID doesn't match IMU_ID.
  */
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS  imu_status;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read Device ID register */
imu_status = read_imu_regs( IMU_REG_CHIP_ID, pdevice_id, sizeof( uint8_t ) );

if ( *pdevice_id != IMU_ID )
    {
    imu_status = IMU_UNRECOGNIZED_OP;
    }

return imu_status;
} /* imu_get_device_id */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

/**
  * @brief  Initialize the magnetometer.
  *
  * @copyright Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
  *
  *         This function is heavily derived from the official Bosch BMM150
  *         driver, which is protected by the BSD-3-Clause license. This
  *         function is exempt from any licensing that may be applied to a
  *         current/future Sun Devil Rocketry project. Per the terms of the
  *         BSD-3-Clause license, the following notice is retained from the
  *         source project and applies to the procedure below.
  *
  *         BSD-3-Clause
  *
  *         Redistribution and use in source and binary forms, with or
  *         without modification, are permitted provided that the following
  *         conditions are met:
  *
  *         1. Redistributions of source code must retain the above
  *         copyright notice, this list of conditions and the following
  *         disclaimer.
  *
  *         2. Redistributions in binary form must reproduce the above
  *         copyright notice, this list of conditions and the following
  *         disclaimer in the documentation and/or other materials provided
  *         with the distribution.
  *
  *         3. Neither the name of the copyright holder nor the names of its
  *         contributors may be used to endorse or promote products derived
  *         from this software without specific prior written permission.
  *
  *         THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  *         CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  *         INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  *         MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  *         DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  *         CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  *         SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  *         LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
  *         USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  *         AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *         LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *         ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *         POSSIBILITY OF SUCH DAMAGE.
  *
  * @param  imu_config_ptr: Pointer to user configuration struct.
  * @return IMU_STATUS
  */
static IMU_STATUS mag_init
    (
    IMU_CONFIG* imu_config_ptr
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
IMU_STATUS imu_status;      /* Status return codes from IMU API     */
uint8_t    device_id;       /* Magnetometer Device ID               */
uint8_t    num_reps_xy_reg; /* Content of XY repetititions register */
uint8_t    num_reps_z_reg;  /* Content of Z repititions register    */
uint8_t    buffer[10];      /* Mag trim read buffer */
uint16_t   temp_msb;        /* Temp variable */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
imu_status      = IMU_OK;
device_id       = 0;
num_reps_xy_reg = ( ( imu_config_ptr -> mag_xy_repititions ) - 1 ) >> 2;
num_reps_z_reg  = ( ( imu_config_ptr -> mag_z_repititions  ) - 1 );


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Put the Magnetometer into sleep mode from suspend mode */
imu_status = write_mag_reg( MAG_REG_PWR_CTRL, 0x01 );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Check Device ID */
HAL_Delay( 5 );
imu_status = read_mag_regs( MAG_REG_CHIP_ID, &device_id, sizeof( device_id ) );
if      ( imu_status != IMU_OK )
    {
    return imu_status;
    }
else if ( device_id != MAG_ID )
    {
    return IMU_MAG_UNRECOGNIZED_ID; 
    }

/* Set the magnetometer operating mode and output data rate */
imu_status = write_mag_reg( MAG_REG_CTRL1, 
                            ( imu_config_ptr -> mag_op_mode ) |
                            ( imu_config_ptr -> mag_odr     ) ); 
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Set the magnetometer measurement repetitions */
imu_status = write_mag_reg( MAG_REG_REP_CTRL_XY, num_reps_xy_reg );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }
imu_status = write_mag_reg( MAG_REG_REP_CTRL_Z, num_reps_z_reg );
if ( imu_status != IMU_OK )
    {
    return imu_status;
    }

/* Set magnetometer trim */

/* ---- Read X1, Y1 ---- */
imu_status = read_mag_regs(MAG_TRIM_REG_X1, buffer, 2);
if ( imu_status != IMU_OK ) 
    {
    return imu_status;
    }
mag_trim.dig_x1 = (int8_t)buffer[0];
mag_trim.dig_y1 = (int8_t)buffer[1];

/* ---- Read Z4_LSB -> Z4_MSB and X2,Y2 ---- */
imu_status = read_mag_regs(MAG_TRIM_REG_Z4_LSB, buffer, 4);
if ( imu_status != IMU_OK ) 
    {
    return imu_status;
    }
mag_trim.dig_z4 = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
mag_trim.dig_x2 = (int8_t)buffer[2];
mag_trim.dig_y2 = (int8_t)buffer[3];

/* ---- Read Z2_LSB -> Z1_MSB (10 bytes) ---- */
imu_status = read_mag_regs(MAG_TRIM_REG_Z2_LSB, buffer, 10);
if ( imu_status != IMU_OK ) 
    {
    return imu_status;
    }
temp_msb = ((uint16_t)buffer[3]) << 8;
mag_trim.dig_z1 = (uint16_t)(temp_msb | buffer[2]);
temp_msb = ((uint16_t)buffer[1]) << 8;
mag_trim.dig_z2 = (int16_t)(temp_msb | buffer[0]);
temp_msb = ((uint16_t)buffer[7]) << 8;
mag_trim.dig_z3 = (int16_t)(temp_msb | buffer[6]);
mag_trim.dig_xy1 = buffer[9];
mag_trim.dig_xy2 = (int8_t)buffer[8];
temp_msb = ((uint16_t)(buffer[5] & 0x7F)) << 8;
mag_trim.dig_xyz1 = (uint16_t)(temp_msb | buffer[4]);

/* Successful magnetometer Initialization */
return IMU_OK;
} /* mag_init */


/**
  * @brief  Read the specified number of registers at one time from the
  *         magnetometer module in the IMU (blocking).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return IMU_STATUS
  */
static IMU_STATUS read_mag_regs 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status return code of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C registers */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_MAG_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_IMU_TIMEOUT );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
    {
    return IMU_MAG_ERROR;
    }
else 
    {
    return IMU_OK;
    }

} /* read_mag_regs */


/**
  * @brief  Read the specified number of registers at one time from the
  *         accelerometer and gyroscope module in the IMU (blocking).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return IMU_STATUS
  */
static IMU_STATUS read_imu_regs 
    (
    uint8_t  reg_addr, /* Register address            */
    uint8_t* data_ptr, /* Register data               */ 
    uint8_t  num_regs  /* Number of registers to read */
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read I2C register */
hal_status = HAL_I2C_Mem_Read( &( IMU_I2C )        , 
                               IMU_ADDR            , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_MAX_DELAY );

if ( hal_status != HAL_OK )
    {
    return IMU_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* read_imu_regs */


/** @brief  Returns the imu_data_ready flag.                                 */
bool imu_get_imu_data_ready
    (
    void
    ) 
{
return imu_data_ready;

} /* imu_get_imu_data_ready */


/** @brief  Returns the mag_data_ready flag.                                 */
bool imu_get_mag_data_ready
    (
    void
    ) 
{
return mag_data_ready;

} /* imu_get_mag_data_ready */


/**
  * @brief  Write to a specified IMU register (blocking).
  * @param  reg_addr: Register address.
  * @param  data:     Register data.
  * @return IMU_STATUS
  */
static IMU_STATUS write_imu_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_IMU_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_reg */


/**
  * @brief  Write to specified IMU registers (blocking).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Source data buffer.
  * @param  num_regs: Number of registers to write.
  * @return IMU_STATUS
  */
static IMU_STATUS write_imu_regs 
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint32_t num_regs  /* Number of registers */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C ), 
                                IMU_ADDR            , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                data_ptr            , 
                                num_regs            , 
                                HAL_MAX_DELAY );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_imu_regs */


/**
  * @brief  Write to a specified magnetometer register (blocking).
  * @param  reg_addr: Register address.
  * @param  data:     Register data.
  * @return IMU_STATUS
  */
static IMU_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ) 
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write( &( IMU_I2C )        , 
                                IMU_MAG_ADDR        , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_IMU_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return IMU_I2C_ERROR;
    }
else
    {
    return IMU_OK;
    }
} /* write_mag_reg */


/**
  * @brief  Kicks off a non-blocking read of the accel and gyro registers.
  * @note   Clears imu_data_ready/mag_data_ready; imu_it_handler() sets them
  *         once the chained accel/gyro + mag reads have both completed.
  * @return IMU_STATUS
  */
IMU_STATUS start_imu_read_IT
    (
    void
    )
{
imu_data_ready = false;
mag_data_ready = false;
return read_imu_regs_IT( IMU_REG_DATA_8, 
                         imu_raw_buffer, 
                         sizeof( imu_raw_buffer ) );
} /* start_imu_read_IT */


/**
  * @brief  Read the IMU registers in interrupt mode.
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return IMU_STATUS
  */
static IMU_STATUS read_imu_regs_IT
    (
    uint8_t  reg_addr, /* Register address            */
    uint8_t* data_ptr, /* Register data               */ 
    uint8_t  num_regs  /* Number of registers to read */
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read I2C register */
hal_status = HAL_I2C_Mem_Read_IT( &( IMU_I2C )        , 
                               IMU_ADDR            , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            );

if ( hal_status != HAL_OK )
    {
    return IMU_ERROR;
    }
else
    {
    return IMU_OK;
    }
}


/**
  * @brief  Handle a memory_rx interrupt signal from the IMU.
  * @note   Two-stage state machine driven by imu_data_ready/mag_data_ready:
  *           1. First call (neither flag set): parses the just-completed
  *              accel/gyro burst into imu_raw_processed, marks
  *              imu_data_ready, then kicks off the chained mag read.
  *           2. Second call (accel/gyro ready, mag not yet): parses the
  *              completed mag burst, sign-extends the 13/15-bit fields,
  *              and marks mag_data_ready.
  * @return IMU_STATUS
  */
IMU_STATUS imu_it_handler
    (
    void
    )
{
IMU_STATUS imu_status = IMU_OK;
if( !imu_data_ready && !mag_data_ready )
    {
    /*------------------------------------------------------------------------------
    Handle IT signal
    ------------------------------------------------------------------------------*/
    /* Combine high byte and low byte to 16 bit data  */
    imu_raw_processed.accel_x = (int16_t) ( (uint16_t) imu_raw_buffer[1] << 8 | imu_raw_buffer[0] );
    imu_raw_processed.accel_y = (int16_t) ( (uint16_t) imu_raw_buffer[3] << 8 | imu_raw_buffer[2] );
    imu_raw_processed.accel_z = (int16_t) ( (uint16_t) imu_raw_buffer[5] << 8 | imu_raw_buffer[4] );
    imu_raw_processed.gyro_x  = (int16_t) ( (uint16_t) imu_raw_buffer[7] << 8 | imu_raw_buffer[6] );
    imu_raw_processed.gyro_y  = (int16_t) ( (uint16_t) imu_raw_buffer[9] << 8 | imu_raw_buffer[8] );
    imu_raw_processed.gyro_z  = (int16_t) ( (uint16_t) imu_raw_buffer[11] << 8 | imu_raw_buffer[10] );

    imu_data_ready = true;

    /*------------------------------------------------------------------------------
    Trigger mag read
    ------------------------------------------------------------------------------*/
    imu_status = read_mag_regs_IT( MAG_REG_DATAX_L, 
                                mag_raw_buffer, 
                                8 );
    return imu_status;
    }
else if( imu_data_ready && !mag_data_ready )
    {
    imu_raw_processed.mag_x
               = (   (uint16_t) mag_raw_buffer[1]                         << MAG_XY_MSB_BITSHIFT ) | 
                 ( ( (uint16_t) mag_raw_buffer[0] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
    imu_raw_processed.mag_y  
               = (   (uint16_t) mag_raw_buffer[3]                         << MAG_XY_MSB_BITSHIFT ) | 
                 ( ( (uint16_t) mag_raw_buffer[2] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
    imu_raw_processed.mag_z  
               = (   (uint16_t) mag_raw_buffer[5]                         << MAG_Z_MSB_BITSHIFT  ) | 
                 ( ( (uint16_t) mag_raw_buffer[4] & MAG_Z_LSB_BITMASK )  >> MAG_Z_LSB_BITSHIFT  );
    imu_raw_processed.mag_hall  
               = (   (uint16_t) mag_raw_buffer[7]                         << MAG_RHALL_MSB_BITSHIFT  ) | 
                 ( ( (uint16_t) mag_raw_buffer[6] & MAG_RHALL_LSB_BITMASK )  >> MAG_RHALL_LSB_BITSHIFT  );

    /* Sign-extend 13-bit and 15-bit values */
    if (imu_raw_processed.mag_x & (1 << 12)) 
        {
        imu_raw_processed.mag_x |= ~((1 << 13) - 1);
        }
    if (imu_raw_processed.mag_y & (1 << 12)) 
        {
        imu_raw_processed.mag_y |= ~((1 << 13) - 1);
        }
    if (imu_raw_processed.mag_z & (1 << 14)) 
        {
        imu_raw_processed.mag_z |= ~((1 << 15) - 1);
        }
    
    mag_data_ready = true;

    return imu_status;
    }

return IMU_FAIL;
} /* imu_it_handler */


/**
  * @brief  Getter function for IMU sensor data (interrupt mode).
  * @param  cpy_ptr: Destination struct for the copied sensor readout.
  * @return IMU_STATUS: IMU_BUSY until both imu_data_ready and mag_data_ready
  *         are set by imu_it_handler().
  */
IMU_STATUS get_imu_it
    (
    IMU_RAW* cpy_ptr
    )
{
if( !imu_data_ready || !mag_data_ready )
    {
    return IMU_BUSY;
    }

memcpy( cpy_ptr, &imu_raw_processed, sizeof( IMU_RAW ) );

return IMU_OK;
} /* get_imu_it */


/**
  * @brief  Read the specified number of registers at one time from the
  *         magnetometer module in the IMU (interrupt mode).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return IMU_STATUS
  */
static IMU_STATUS read_mag_regs_IT 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status return code of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C registers */
hal_status = HAL_I2C_Mem_Read_IT( &( IMU_I2C )     , 
                               IMU_MAG_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
    {
    return IMU_MAG_ERROR;
    }
else 
    {
    return IMU_OK;
    }

} /* read_mag_regs_IT */


/**
  * @brief  Getter function for the magnetometer trim from mag_init().
  * @return MAG_TRIM
  */
MAG_TRIM imu_get_mag_trim
    (
    void
    )
{
return mag_trim;

} /* imu_get_mag_trim */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
