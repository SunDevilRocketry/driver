/**
  ******************************************************************************
  * @file           : mag.c
  * @brief          : Driver for rev 3's BMM350 magnetometer.
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

// TODO add attribution to Bosch's official BMM350 driver

/* Includes ------------------------------------------------------------------*/

/* Standard */
#include <stdint.h>

/* Project */
#include "mag.h"
#include "sdr_pin_defines_A0010.h"

/* Definitions ----------------------------------------------------------*/
#define MAG_I2C_ADDR 0x00 << 1 // TODO check if correct
#define MAG_TIMEOUT  10
#define MAG_BUFFER_LEN 12 /* Byte length of a sensor read */

#define MAG_OTP_ADDR_MASK 0x1F
#define MAG_OTP_LEN       32
#define MAG_OTP_CMD_DONE  0x01
#define MAG_OTP_ERR_MASK  0xE0

// If we implement more CMDs, turn this into a struct
#define MAG_CMD_OTP_READ = 0x20

// TODO define all of them
typedef enum _MAG_REG
    {
    MAG_REG_PADCTRL    = 0x03,
    MAG_REG_X_XLSB     = 0x31, /* Starting addr of sensor data */
    MAG_REG_OTP_MSB    = 0x52,
    MAG_REG_OTP_STATUS = 0x55,
    MAG_REG_CMD        = 0x7E
    } MAG_REG;

typedef enum _MAG_CMD
    {
    
    } MAG_CMD;
    
/* Global Variables ----------------------------------------------------------*/
static uint16_t mag_otp[MAG_OTP_LEN];

static MAG_STATUS ready = MAG_BUSY;

static MAG_XYZ *mag_ptr;

static uint8_t mag_read_buffer[MAG_BUFFER_LEN];

/* Private function prototypes -----------------------------------------------*/
static MAG_STATUS write_reg
    (
    uint8_t *buffer,
    uint8_t size,
    MAG_REG address
    );

static MAG_STATUS read_reg
    (
    uint8_t *buffer,
    uint8_t size,
    MAG_REG address
    );

static MAG_STATUS otp_reg_to_err_stat
    (
    uint8_t status,
    MAG_STATUS default
    );

/* Procedures ----------------------------------------------------------------*/

/**
 * @brief Initialize the magnetometer
 *
 * @details Restarts the magnetometer and applies the input config
 *
 * @retval The status of the magnetometer
 */
MAG_STATUS mag_init
    (
    MAG_CONFIG *mag_config_ptr
    )
{
/* Local variables */
uint8_t config_buffer[3]; /* Stores the registers that will be written over I2C */

HAL_StatusTypeDef success;

/* Implementation */
config_buffer[0] = mag_config_ptr->pad_ctrl;

config_buffer[1] = (mag_config_ptr->avg<< 4) |
                    mag_config_ptr->odr;

config_buffer[2] = (mag_config_ptr->axis_z << 2) |
                   (mag_config_ptr->axis_y << 1) |
                    mag_config_ptr->axis_x;

success = write_reg(config_buffer, 3, MAG_REG_PADCTRL);

if(success != HAL_OK) {
    return MAG_I2C_FAIL;
}

for(int addr = 0; addr < MAG_OTP_LEN; addr++) {
    uint8_t read_cmd = MAG_CMD_OTP_READ | (addr & MAG_OTP_ADDR_MASK);
    
    success = write_reg(&read_cmd, 1, MAG_REG_CMD);

    if(success != HAL_OK) {
        return MAG_I2C_FAIL;
    }
    
    /* Poll OTP status register */
    uint8_t status = 0x00;
    
    while( !(status & MAG_OTP_CMD_DONE) ) {
       success = read_reg(&status, 1, MAG_REG_OTP_STATUS);
       
       if(success != HAL_OK) {
           return MAG_I2C_FAIL;
       }

       /* Check status */
       if(status & MAG_OTP_ERR_MASK != 0) {
           /* Internal modem error */
           return otp_reg_to_err_stat(status, MAG_INIT_FAIL);
       }
    } 

    /* Read the constant */
    uint8_t otp_const[2];
    
    success = read_reg(otp_const, 2, MAG_REG_OTP_MSB);

    if(success != HAL_OK) {
        return MAG_I2C_FAIL;
    }

    /* Concatenate two bytes in array into a uint16_t */
    mag_otp[addr] = (otp_const[0] << 8) | otp_cost[1];
}

return MAG_OK;
}

/**
 * @brief See if most recently-requested mag read is ready
 *
 * @details Reads global ready variable set by mag_get_xyz (starts
 * IT I2C read) and mag_it_handler
 *
 * @retval MAG_OK if ready, MAG_BUSY if waiting, MAG_I2C_FAIL
 * if I2C read fails (only these values are allowed)
 */
MAG_STATUS mag_get_ready
    (
    void
    )
{
debug_assert(true, ready == MAG_BUSY     |
                   ready == MAG_OK       |
                   ready == MAG_I2C_FAIL );
return ready;
}

/**
 * @brief Initiate interrupt-based magnetometer read
 *
 * @details Uses HAL I2C IT call to read relevant sensors,
 * and sets the ready varable accordingly
 *
 * @retval MAG_OK if successful, relevant error condition
 * otherwise
 */
MAG_STATUS mag_get_xyz
    (
    MAG_XYZ *mag_xyz
    )
{
/* Local variables */
HAL_StatusTypeDef success;

/* Implementation */
mag_ptr = mag_xyz;

success = HAL_I2C_Mem_Write_IT(&(MAG_I2C)           ,
                                MAG_I2C_ADDR        ,
                                MAG_REG_X_XLSB      ,
                                I2C_MEMADD_SIZE_8BIT,
                                mag_read_buffer     ,
                                MAG_BUFFER_LEN);

if(success == HAL_OK) {
    ready = MAG_BUSY;
    return MAG_OK;
} else {
    ready = MAG_I2C_FAIL;
    return MAG_I2C_FAIL
}
}

/* Private functions -----------------------------------------------*/

/* Blocking magnetometer register write */
/* Only use this in init code */
static MAG_STATUS write_reg
    (
    uint8_t *buffer,
    uint8_t size,
    MAG_REG address
    )
{
HAL_StatusTypeDef success;
success = HAL_I2C_Mem_Write(&(MAG_I2C)          ,
                            MAG_I2C_ADDR        ,
                            address             ,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer              ,
                            size                ,
                            MAG_TIMEOUT );
if(success == HAL_OK) {
    return MAG_OK;
} else {
    return MAG_I2C_FAIL;
}
}

/* Blocking read, only for init */
static MAG_STATUS read_reg
    (
    uint8_t *buffer,
    uint8_t size,
    MAG_REG address
    )
{
HAL_StatusTypeDef success;
success = HAL_I2C_Mem_Read(&(MAG_I2C)          ,
                            MAG_I2C_ADDR        ,
                            address             ,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer              ,
                            size                ,
                            MAG_TIMEOUT );

if(success == HAL_OK) {
    return MAG_OK;
} else {
    return MAG_I2C_FAIL;
}
}

static MAG_STATUS otp_reg_to_err_stat
    (
    uint8_t status,
    MAG_STATUS default
    )
{
switch( (status & MAG_OTP_ERR_MASK) >> 5 ) {
    case 1:
        return MAG_BOOT_FAIL;
    case 2:
        return MAG_PAGE_READ_FAIL;
    case 3:
        return MAG_PAGE_PROG_FAIL;
    case 4:
        return MAG_SIGN_ERR;
    case 5:
        return MAG_INV_CMD_ERR;
    default:
        return default;
}
}
