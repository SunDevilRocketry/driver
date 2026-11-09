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
#include "main.h"
#include "mag.h"
#include "sdr_pin_defines_A0010.h"
#include "error_sdr.h"
#include "debug_sdr.h"

/* Function-like Macros -------------------------------------------------*/
/* Concatenates 3 bytes from the read data */
#define bytes_to_int(i) ( mag_read_buffer[i + 2] << 16 ) | \
                        ( mag_read_buffer[i + 1] << 8  ) | \
                          mag_read_buffer[i]

/* Definitions ----------------------------------------------------------*/
#define MAG_I2C_ADDR 0x00 << 1 // TODO check if correct
#define MAG_TIMEOUT  10
#define MAG_BUFFER_LEN 12 /* Byte length of a sensor read */

#define MAG_OTP_ADDR_MASK 0x1F
#define MAG_OTP_LEN       32
#define MAG_OTP_CMD_DONE  0x01
#define MAG_OTP_ERR_MASK  0xE0
#define MAG_OTP_SENS_X    0x10
#define MAG_OTP_SENS_Y    0x11
#define MAG_OTP_SENS_Z    0x11
#define MAG_OTP_SENS_T    0x0D


// If we implement more CMDs, turn this into a struct
#define MAG_CMD_OTP_READ 0x20

// TODO define all of them
typedef enum _MAG_REG
    {
    MAG_REG_PADCTRL    = 0x03,
    MAG_REG_X_XLSB     = 0x31, /* Starting addr of sensor data */
    MAG_REG_OTP_MSB    = 0x52,
    MAG_REG_OTP_STATUS = 0x55,
    MAG_REG_CMD        = 0x7E
    } MAG_REG;
    
/* Global Variables ----------------------------------------------------------*/
static uint16_t mag_otp[MAG_OTP_LEN];

static MAG_STATUS ready = MAG_BUSY;

static uint8_t mag_read_buffer[MAG_BUFFER_LEN];

static float lsb_to_ut_degc[4];

/* Holds OTP constants converted into useful forms */
// TODO consider putting these into a struct like Bosch does
static float sens_x, sens_y, sens_z, t_sens;
static float t_offs, dut_t0;
static float dut_offset_coef[3], dut_sensit_coef[3], dut_tco[3], dut_tcs[3];
static float cross_x_y, cross_y_x, cross_z_y, cross_z_x;

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
    MAG_STATUS default_status
    );

static void update_default_coefiecents
    (
    float *lsb_to_ut_degc
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

if ( success != HAL_OK ) 
    {
    return MAG_I2C_FAIL;
    }

for ( int addr = 0; addr < MAG_OTP_LEN; addr++ )
    {
    uint8_t read_cmd = MAG_CMD_OTP_READ | (addr & MAG_OTP_ADDR_MASK);
    
    success = write_reg(&read_cmd, 1, MAG_REG_CMD);

    if( success != HAL_OK )
        {
        return MAG_I2C_FAIL;
        }
    
    /* Poll OTP status register */
    uint8_t status = 0x00;
    
    while ( !(status & MAG_OTP_CMD_DONE) )
        {
       success = read_reg(&status, 1, MAG_REG_OTP_STATUS);
       
       if ( success != HAL_OK )
        {
           return MAG_I2C_FAIL;
        }

       /* Check status */
       if ( ( status & MAG_OTP_ERR_MASK ) != 0 )
            {
           /* Internal modem error */
           return otp_reg_to_err_stat(status, MAG_INIT_FAIL);
            }
        } 

    /* Read the constant */
    uint8_t otp_const[2];
    
    success = read_reg(otp_const, 2, MAG_REG_OTP_MSB);

    if ( success != HAL_OK )
        {
        return MAG_I2C_FAIL;
        }

    /* Concatenate two bytes in array into a uint16_t */
    mag_otp[addr] = (otp_const[0] << 8) | otp_const[1];
    }

/* Initialize some needed conversion constants */
update_default_coefiecents( lsb_to_ut_degc );

/* Turn the OTP data into some useful coefficients */
/* Taken from official driver function update_mag_off_sens */
/* TODO make sure this type cast actually does the same thing as Bosch fix_sign */
sens_x = ( (int8_t) ( ( mag_otp[MAG_OTP_SENS_X] & 0xFF00 ) >> 8 ) ) / 256.0f;
sens_y = ( (int8_t) ( mag_otp[MAG_OTP_SENS_Y] & 0x00FF ) ) / 256.0f;
sens_z = ( (int8_t) ( ( mag_otp[MAG_OTP_SENS_Z] & 0xFF00 ) >> 8 ) ) / 256.0f;
t_sens = ( (int8_t) ( mag_otp[MAG_OTP_SENS_T] & 0xFF00 ) >> 8 ) / 256.0f;

t_offs = ( (int8_t) ( mag_otp[MAG_OTP_SENS_T] & 0x00FF ) ) / 5.0f;

// TODO initialize other constants needed for compensation

return MAG_OK;

} /* mag_init */

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
debug_assert((ready == MAG_BUSY | ready == MAG_OK | ready == MAG_I2C_FAIL), ERROR_UNKNOWN_FATAL_ERROR);
// TODO update error.h with its own error
return ready;

} /* mag_get_ready */

/**
 * @brief Initiate interrupt-based magnetometer read
 *
 * @details Uses HAL I2C IT call to read relevant sensors,
 * and sets the ready varable accordingly
 *
 * @retval MAG_OK if successful, relevant error condition
 * otherwise
 */
MAG_STATUS mag_start_read
    (
    void
    )
{
/* Local variables */
HAL_StatusTypeDef success;

/* Implementation */
success = HAL_I2C_Mem_Write_IT(&(MAG_I2C)           ,
                                MAG_I2C_ADDR        ,
                                MAG_REG_X_XLSB      ,
                                I2C_MEMADD_SIZE_8BIT,
                                mag_read_buffer     ,
                                MAG_BUFFER_LEN);

if ( success == HAL_OK )
    {
    ready = MAG_BUSY;
    return MAG_OK;
    }

ready = MAG_I2C_FAIL;
return MAG_I2C_FAIL;

} /* mag_start_read */ 

/**
 * @brief Finish off the magnetomer read.
 *
 * @details Basically just lets us know a read is complete.
 * Most of the heavy lifting is done in get_xyz.
 *
 * @retval Status. This status shouldn't matter because
 * it's an interrupt handler, but other libraries tend to
 * do it anyway.
 */
MAG_STATUS mag_it_handler
    (
    void
    )
{
if ( ready != MAG_OK )
    {
    ready = MAG_OK;
    return MAG_OK;
    }

return MAG_I2C_FAIL;

} /* mag_it_handler */

/**
 * @brief Run conversion of raw sensor data into XYZ struct
 *
 * @details A whole ton of math, based on the official
 * Bosch driver.
 * 
 * @param Pointer to the XYZ struct that the result data should
 * be put in.
 *
 * @retval The status of the conversion. Only situation for failure
 * should be if this is initiated before ready is in the MAG_OK
 * state.
 */
MAG_STATUS mag_get_xyz
    (
    MAG_XYZ *mag_xyz
    )
{
/* Local variables */
MAG_STATUS ready_val;

uint32_t xint, yint, zint, tint;

float temp;

float* out_data[3] = { &(mag_xyz->mag_x),
                      &(mag_xyz->mag_y),
                      &(mag_xyz->mag_z) };

float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;

/* Implementation */
ready_val = mag_get_ready();
if ( ready == MAG_OK )
    {
    /* Concanate bytes of raw data */
    xint = bytes_to_int(0);
    yint = bytes_to_int(3);
    zint = bytes_to_int(6);
    tint = bytes_to_int(9);

    /* Get compensated temperature, used for comp of other variables */
    temp = ( 1 + t_sens ) * tint + t_offs;

    /* Compensate data */
    for ( int i = 0; i < 3; i++)
    {
        *out_data[i] = 0.0f;
        *out_data[i] *= 1 + dut_sensit_coef[i];
        *out_data[i] += dut_offset_coef[i];
        *out_data[i] += dut_tco[i] * (temp - dut_t0);
        *out_data[i] /= 1 + dut_tcs[i] * (temp - dut_t0);
        // TODO make sure we initialize dut_t0 in mag_init
    }

    /* Cross axis compensation */
    cr_ax_comp_x = (*out_data[0] - cross_x_y * *out_data[1]) /
                   (1 - cross_y_x * cross_x_y);
    cr_ax_comp_y = (*out_data[1] - cross_y_x * *out_data[0]) /
                   (1 - cross_y_x * cross_x_y);

    // TODO this looks like there has been child murder.
    // Comprehend this, and then make it look less ugly.
    cr_ax_comp_z =
        (*out_data[2] +
        (*out_data[0] *
          (cross_y_x * cross_z_y -
           cross_z_x) - *out_data[1] *
          (cross_z_y - cross_x_y *
           cross_z_x)) /
         (1 - cross_y_x * cross_x_y));

    *out_data[0] = cr_ax_comp_x;
    *out_data[1] = cr_ax_comp_y;
    *out_data[2] = cr_ax_comp_z;
    
    return MAG_OK;
    }

return ready_val;

} /* mag_get_xyz */

/* Private functions -----------------------------------------------*/

/**
 * @brief Blocking magnetometer register write
 * 
 * @warning Only use this in initialization code; all other writes
 * are done using non-blocking functions.
 *
 * @details Wraps around STM32 HAL function, with I2C parameters
 * baked in and HAL return types converted to MAG_STATUS.
 * 
 * @param buffer Buffer to write I2C bytes from
 * @param size Size of buffer
 * @param address I2C register address to write to.
 *
 * @retval Status of I2C read
 */
static MAG_STATUS write_reg
    (
    uint8_t* buffer,
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
if ( success == HAL_OK )
    {
    return MAG_OK;
    }

return MAG_I2C_FAIL;

} /* write_reg */

/**
 * @brief Blocking magnetometer register read
 * 
 * @warning Only use this in initialization code; all sensor writes
 * are done using non-blocking functions.
 *
 * @details Wraps around STM32 HAL function, with I2C parameters
 * baked in and HAL return types converted to MAG_STATUS.
 * 
 * @param buffer Buffer to read I2C bytes into
 * @param size Size of buffer
 * @param address I2C register address to read from.
 *
 * @retval Status of I2C read
 */
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

if ( success == HAL_OK )
    {
    return MAG_OK;
    }

return MAG_I2C_FAIL;

} /* read_reg */

/**
 * @brief Converts OTP status to one of our error ENUM values
 * 
 * @param status Raw OTP status
 * @param default_status Default status to return if not matched
 *
 * @retval Result of status conversion.
 */
static MAG_STATUS otp_reg_to_err_stat
    (
    uint8_t status,
    MAG_STATUS default_status
    )
{
switch( (status & MAG_OTP_ERR_MASK) >> 5 )
    {
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
        return default_status;
    }
    
} /* otp_reg_to_err_stat */

/**
 * @brief Calculate some coefficients needed for conversions.
 *
 * @details Copied verbatim from the official BM350 driver,
 * minus a few stylistic modifications.
 * 
 * @param lsb_to_ut_degc Pointer to array of said coefficients.
 */
static void update_default_coefiecents
    (
    float* lsb_to_ut_degc
    )
{
/* Local Variables */
float bxy_sens, bz_sens, temp_sens, ina_xy_gain_trgt, ina_z_gain_trgt, adc_gain, lut_gain;
float power;

/* Implementation */
bxy_sens = 14.55f;
bz_sens = 9.0f;
temp_sens = 0.00204f;

ina_xy_gain_trgt = 19.46f;

ina_z_gain_trgt = 31.0;

adc_gain = 1 / 1.5f;
lut_gain = 0.714607238769531f;

power = (float)(1000000.0 / 1048576.0);

lsb_to_ut_degc[0] = (power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
lsb_to_ut_degc[1] = (power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
lsb_to_ut_degc[2] = (power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain));
lsb_to_ut_degc[3] = 1 / (temp_sens * adc_gain * lut_gain * 1048576);

} /* update_default_coefiecents */
