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
#include <stdatomic.h>


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
/* Timeouts */
#define HAL_MAG_TIMEOUT             10

/** @brief Magnetometer (BMM150) I2C address, pre-shifted for HAL addressing  */


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

/* BMM150 */
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
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Static Variables 
------------------------------------------------------------------------------*/
/** @brief Raw double-buffer target for interrupt-mode magnetometer reads     */
static uint8_t mag_raw_buffer[8];
/** @brief Parsed interrupt-mode mag sample, published by mag_it_handler() */
static MAG_RAW mag_raw_processed;
/** @brief Set true by mag_it_handler() once mag_raw_processed's fields are valid */
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

/** @brief Read Magnetometer registers (blocking)                            */
static MAG_STATUS read_mag_regs
    (
    uint8_t  reg_addr, /* Register address    */
    uint8_t* data_ptr, /* Register data       */
    uint8_t  num_regs  /* Number of registers */
    ); 

/** @brief Write to a specified magnetometer register (blocking)             */
static MAG_STATUS write_mag_reg 
    (
    uint8_t reg_addr, /* Register address    */
    uint8_t data      /* Register data       */
    ); 

/** @brief Read Magnetometer registers (interrupt mode)                      */
static MAG_STATUS read_mag_regs_IT 
    (
    uint8_t  reg_addr,
    uint8_t* data_ptr, 
    uint8_t  num_regs
    );


/*------------------------------------------------------------------------------
 Procedures 
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
  * @param  mag_config_ptr: Pointer to user configuration struct.
  * @return MAG_STATUS
  */
MAG_STATUS mag_init
    (
    MAG_CONFIG* mag_config_ptr
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
MAG_STATUS mag_status;      /* Status return codes from mag API     */
uint8_t    device_id;       /* Magnetometer Device ID               */
uint8_t    num_reps_xy_reg; /* Content of XY repetititions register */
uint8_t    num_reps_z_reg;  /* Content of Z repititions register    */
uint8_t    buffer[10];      /* Mag trim read buffer */
uint16_t   temp_msb;        /* Temp variable */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
mag_status      = MAG_OK;
device_id       = 0;
num_reps_xy_reg = ( ( mag_config_ptr->xy_repititions ) - 1 ) >> 2;
num_reps_z_reg  = ( ( mag_config_ptr->z_repititions  ) - 1 );

/* clear double buffer if using mag with IT */
memset( &mag_raw_buffer, 0, sizeof( mag_raw_buffer ) );
mag_data_ready = false;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* disable interrupts while initializing */
HAL_NVIC_DisableIRQ(IMU_MAG_I2C_EV_IRQn);

/* Put the Magnetometer into sleep mode from suspend mode */
mag_status = write_mag_reg( MAG_REG_PWR_CTRL, 0x01 );
if ( mag_status != MAG_OK )
    {
    return mag_status;
    }

/* Check Device ID */
HAL_Delay( 5 );
mag_status = read_mag_regs( MAG_REG_CHIP_ID, &device_id, sizeof( device_id ) );
if ( mag_status != IMU_OK )
    {
    return mag_status;
    }
else if ( device_id != MAG_OK )
    {
    return MAG_UNRECOGNIZED_ID; 
    }

/* Set the magnetometer operating mode and output data rate */
mag_status = write_mag_reg( MAG_REG_CTRL1, 
                            ( mag_config_ptr->op_mode ) |
                            ( mag_config_ptr->odr     ) ); 
if ( mag_status != MAG_OK )
    {
    return mag_status;
    }

/* Set the magnetometer measurement repetitions */
mag_status = write_mag_reg( MAG_REG_REP_CTRL_XY, num_reps_xy_reg );
if ( mag_status != MAG_OK )
    {
    return mag_status;
    }
mag_status = write_mag_reg( MAG_REG_REP_CTRL_Z, num_reps_z_reg );
if ( mag_status != MAG_OK )
    {
    return mag_status;
    }

/* Set magnetometer trim */

/* ---- Read X1, Y1 ---- */
mag_status = read_mag_regs(MAG_TRIM_REG_X1, buffer, 2);
if ( mag_status != MAG_OK ) 
    {
    return mag_status;
    }
mag_trim.dig_x1 = (int8_t)buffer[0];
mag_trim.dig_y1 = (int8_t)buffer[1];

/* ---- Read Z4_LSB -> Z4_MSB and X2,Y2 ---- */
mag_status = read_mag_regs(MAG_TRIM_REG_Z4_LSB, buffer, 4);
if ( mag_status != MAG_OK ) 
    {
    return mag_status;
    }
mag_trim.dig_z4 = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
mag_trim.dig_x2 = (int8_t)buffer[2];
mag_trim.dig_y2 = (int8_t)buffer[3];

/* ---- Read Z2_LSB -> Z1_MSB (10 bytes) ---- */
mag_status = read_mag_regs(MAG_TRIM_REG_Z2_LSB, buffer, 10);
if ( mag_status != MAG_OK ) 
    {
    return mag_status;
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

/* re-enable interrupts while initializing */
HAL_NVIC_EnableIRQ(IMU_MAG_I2C_EV_IRQn);

/* Successful magnetometer Initialization */
return MAG_OK;

} /* mag_init */


/** @brief  Returns the mag_data_ready flag.                                 */
bool mag_get_data_ready
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


/**
 * @brief  Blocking read of magnetometer X/Y/Z.
 * @param[out] mag_raw: Destination struct for the magnetometer readout.
 * @return MAG_STATUS
 */
MAG_STATUS mag_get_xyz
    (
    MAG_RAW* mag_raw
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
MAG_STATUS mag_status;     /* Mag status return codes           */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers */
mag_status = read_mag_regs( MAG_REG_DATAX_L, 
                            &regMag[0]     , 
                            sizeof( regMag ) );

/* Check for HAL IMU error */
if ( mag_status == MAG_TIMEOUT )
    {
    return MAG_TIMEOUT;
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
mag_raw->mag_x    = mag_x_raw;
mag_raw->mag_y    = mag_y_raw;
mag_raw->mag_z    = mag_z_raw;
mag_raw->mag_hall = mag_hall_raw;

return MAG_OK;
} /* mag_get_xyz */

/**
  * @brief  Receicve magnetometer data in interrupt mode.
  * @return MAG_STATUS
  */
MAG_STATUS start_mag_read_IT
    (
    void
    )
{
mag_data_ready = false;

return read_mag_regs_IT( IMU_REG_DATA_8, 
                         mag_raw_buffer, 
                         sizeof( mag_raw_buffer ) );

} /* start_mag_read_IT */

/**
  * @brief  Handle a memory_rx interrupt signal from the magnetometer.
  * @return MAG_STATUS
  */
MAG_STATUS mag_it_handler
    (
    void
    )
{
MAG_STATUS mag_status = MAG_OK;

mag_raw_processed.mag_x
            = (   (uint16_t) mag_raw_buffer[1]                         << MAG_XY_MSB_BITSHIFT ) | 
                ( ( (uint16_t) mag_raw_buffer[0] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
mag_raw_processed.mag_y  
            = (   (uint16_t) mag_raw_buffer[3]                         << MAG_XY_MSB_BITSHIFT ) | 
                ( ( (uint16_t) mag_raw_buffer[2] & MAG_XY_LSB_BITMASK ) >> MAG_XY_LSB_BITSHIFT );
mag_raw_processed.mag_z  
            = (   (uint16_t) mag_raw_buffer[5]                         << MAG_Z_MSB_BITSHIFT  ) | 
                ( ( (uint16_t) mag_raw_buffer[4] & MAG_Z_LSB_BITMASK )  >> MAG_Z_LSB_BITSHIFT  );
mag_raw_processed.mag_hall  
            = (   (uint16_t) mag_raw_buffer[7]                         << MAG_RHALL_MSB_BITSHIFT  ) | 
                ( ( (uint16_t) mag_raw_buffer[6] & MAG_RHALL_LSB_BITMASK )  >> MAG_RHALL_LSB_BITSHIFT  );

/* Sign-extend 13-bit and 15-bit values */
if (mag_raw_processed.mag_x & (1 << 12)) 
    {
    mag_raw_processed.mag_x |= ~((1 << 13) - 1);
    }
if (mag_raw_processed.mag_y & (1 << 12)) 
    {
    mag_raw_processed.mag_y |= ~((1 << 13) - 1);
    }
if (mag_raw_processed.mag_z & (1 << 14)) 
    {
    mag_raw_processed.mag_z |= ~((1 << 15) - 1);
    }

mag_data_ready = true;

return mag_status;

}

/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

/**
  * @brief  Read the specified number of registers at one time from the
  *         magnetometer module in the IMU (blocking).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return IMU_STATUS
  */
static MAG_STATUS read_mag_regs 
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
hal_status = HAL_I2C_Mem_Read( &( IMU_MAG_I2C )        , 
                               MAG_I2C_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            , 
                               HAL_MAG_TIMEOUT );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
    {
    return MAG_ERROR;
    }
else 
    {
    return MAG_OK;
    }

} /* read_mag_regs */


/**
  * @brief  Write to a specified magnetometer register (blocking).
  * @param  reg_addr: Register address.
  * @param  data:     Register data.
  * @return MAG_STATUS
  */
static MAG_STATUS write_mag_reg 
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
hal_status = HAL_I2C_Mem_Write( &( IMU_MAG_I2C )        , 
                                MAG_I2C_ADDR        , 
                                reg_addr            , 
                                I2C_MEMADD_SIZE_8BIT, 
                                &data               , 
                                sizeof( uint8_t )   , 
                                HAL_MAG_TIMEOUT );
if ( hal_status != HAL_OK )
    {
    return MAG_I2C_ERROR;
    }
else
    {
    return MAG_OK;
    }
} /* write_mag_reg */


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
mag_trim = mag_get_trim();

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


/**
  * @brief  Read the specified number of registers at one time from the
  *         magnetometer (interrupt mode).
  * @param  reg_addr: Starting register address.
  * @param  data_ptr: Destination buffer.
  * @param  num_regs: Number of registers to read.
  * @return MAG_STATUS
  */
static MAG_STATUS read_mag_regs_IT 
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
hal_status = HAL_I2C_Mem_Read_IT( &( IMU_MAG_I2C ) , 
                               MAG_I2C_ADDR        , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               data_ptr            , 
                               num_regs            );

/* Return status code of I2C HAL */
if ( hal_status != HAL_OK ) 
    {
    return MAG_ERROR;
    }
else 
    {
    return MAG_OK;
    }

} /* read_mag_regs_IT */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
