/**
  ******************************************************************************
  * @file           : bmm150.c
  * @brief          : Bosch BMM150 magnetometer implementation
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


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#include "pindefs.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "mag.h"



/*------------------------------------------------------------------------------
 Private Macros
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Static Variables 
------------------------------------------------------------------------------*/
/** @brief Raw double-buffer target for interrupt-mode magnetometer reads     */
static uint8_t mag_raw_buffer[8];
/** @brief Set true by imu_it_handler() once imu_raw_processed's mag fields are valid */
static atomic_bool mag_data_ready;

/** @brief BMM150 factory trim coefficients, populated by mag_init()          */
static MAG_TRIM mag_trim;

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

static void mag_conv_raw
	(
	MAG_CONVERTED* mag_converted, 
	MAG_RAW* mag_raw
	);


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/** @brief  Returns the mag_data_ready flag.                                 */
bool mag_get_mag_data_ready
    (
    void
    )
{
return mag_data_ready;
} /* imu_get_mag_data_ready */


/**
  * @brief  Getter function for the magnetometer trim from mag_init().
  * @return MAG_TRIM
  */
MAG_TRIM mag_get_trim
    (
    void
    )
{
return mag_trim;
} /* mag_get_trim */

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
static MAG_STATUS mag_init
    (
    MAG_CONFIG* mag_config_ptr
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
  * @brief Converts raw magnetometer readings into magnetic field data.
  * @param[out] imu_converted Converted magnetometer data to update.
  * @param[in]  mag_raw Raw magnetometer readouts.
  *
  * @attention 
  * 
  * This function is heavily derived from the official Bosch BMM150
  * driver, which is protected by the BSD-3-Clause license. This function
  *	is exempt from any licensing that may be applied to a current/future
  *	Sun Devil Rocketry project. Per the terms of the BSD-3-Clause license,
  *	the following notice is retained from the source project and applies
  *	to the procedure below.
  *
  * 	Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
  *
  *		BSD-3-Clause
  *																			   
  *		Redistribution and use in source and binary forms, with or without	   
  *		modification, are permitted provided that the following conditions are 
  *		met:																   
  *																			   
  *		1. Redistributions of source code must retain the above copyright      
  *	    notice, this list of conditions and the following disclaimer.		   
  *																			   
  *		2. Redistributions in binary form must reproduce the above copyright   
  *	    notice, this list of conditions and the following disclaimer in the    
  *	    documentation and/or other materials provided with the distribution.   
  *																			   
  *		3. Neither the name of the copyright holder nor the names of its       
  *	    contributors may be used to endorse or promote products derived from   
  *	    this software without specific prior written permission. 			   
  *																			   
  *		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS	   
  *		"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT	   
  *		LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS	   
  *		FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE		   
  *		COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,   
  *		INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES			   
  *		(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR	   
  *		SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)	   
  *		HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,	   
  *		STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  
  *		IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE	   
  *		POSSIBILITY OF SUCH DAMAGE.		
  */
static void mag_conv_raw
	(
	MAG_CONVERTED* mag_converted, 
	MAG_RAW* mag_raw
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
MAG_TRIM mag_trim;
float mag_x;
float mag_y;
float mag_z;

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
mag_trim = imu_get_mag_trim();

/*------------------------------------------------------------------------------
 Apply Bosch compensation using factory trim values
------------------------------------------------------------------------------*/
float rhall = mag_raw->mag_hall;
if ( (rhall != 0) && (mag_trim.dig_xyz1 != 0) )
    {
    /* ---- X compensation ---- */
    float process_comp_x0 = (((float)mag_trim.dig_xyz1) * 16384.0f / rhall);
    mag_x = (process_comp_x0 - 16384.0f);
    float process_comp_x1 = ((float)mag_trim.dig_xy2) * (mag_x * mag_x / 268435456.0f);
    float process_comp_x2 = process_comp_x1 + mag_x * ((float)mag_trim.dig_xy1) / 16384.0f;
    float process_comp_x3 = ((float)mag_trim.dig_x2) + 160.0f;
    float process_comp_x4 = ((float)mag_raw->mag_x) * ((process_comp_x2 + 256.0f) * process_comp_x3);
    mag_x = ((process_comp_x4 / 8192.0f) + (((float)mag_trim.dig_x1) * 8.0f)) / 16.0f; /* µT */

    /* ---- Y compensation ---- */
    float process_comp_y0 = ((float)mag_trim.dig_xyz1) * 16384.0f / rhall;
    mag_y = process_comp_y0 - 16384.0f;
    float process_comp_y1 = ((float)mag_trim.dig_xy2) * (mag_y * mag_y / 268435456.0f);
    float process_comp_y2 = process_comp_y1 + mag_y * ((float)mag_trim.dig_xy1) / 16384.0f;
    float process_comp_y3 = ((float)mag_trim.dig_y2) + 160.0f;
    float process_comp_y4 = ((float)mag_raw->mag_y) * (((process_comp_y2) + 256.0f) * process_comp_y3);
    mag_y = ((process_comp_y4 / 8192.0f) + (((float)mag_trim.dig_y1) * 8.0f)) / 16.0f; /* µT */

    /* ---- Z compensation ---- */
    float process_comp_z0 = ((float)mag_raw->mag_z) - ((float)mag_trim.dig_z4);
    float process_comp_z1 = ((float)rhall) - ((float)mag_trim.dig_xyz1);
    float process_comp_z2 = (((float)mag_trim.dig_z3) * process_comp_z1);
    float process_comp_z3 = ((float)mag_trim.dig_z1) * ((float)rhall) / 32768.0f;
    float process_comp_z4 = ((float)mag_trim.dig_z2) + process_comp_z3;
    float process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
    mag_z = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f; /* µT */
    }
else /* Data is invalid */
    {
    mag_x = 0.0f;
    mag_y = 0.0f;
    mag_z = 0.0f;
    }
/*------------------------------------------------------------------------------
 Store converted field data
------------------------------------------------------------------------------*/
mag_converted->mag_x = mag_x;
mag_converted->mag_y = mag_y;
mag_converted->mag_z = mag_z;
} /* mag_conv_raw */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
