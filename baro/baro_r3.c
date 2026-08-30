/**
  ******************************************************************************
  * @file           : baro_r3.c
  * @brief          : Driver for the MS5607-02BA03 barometer on FC rev 3.
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

/* Includes ------------------------------------------------------------------*/

/* Standard */
#include <stdbool.h>
#include <stdatomic.h>

/* Project */
#include "main.h"
#include "error_sdr.h"
#include "stm32h7xx_hal.h"
#include "sdr_pin_defines_A0010.h"
#include "baro.h"
#include "debug_sdr.h"

/* Type Definitions ----------------------------------------------------------*/

/* Baro calibration data struct in integer format */
typedef struct _BARO_CAL_DATA_INT
	{
	uint16_t par_c1; // Pressure Sensitity
	uint16_t par_c2; // Pressure Offset
	uint16_t par_c3; // Temp. Coeff. of Pressure Sensitivity
	uint16_t par_c4; // Temp. Coeff. of Pressure Offset
	uint16_t par_c5; // Reference Temperature
	uint16_t par_c6; // Temp. Coeff. of Temperature
	} BARO_CAL_DATA_INT;

/* States for FSM to read data */
typedef enum _BARO_READ_STATE
    {
    BARO_CONV_PRESSURE,    // Start pressure Conversion
    BARO_WAIT_PRESSURE,    // Pressure Conversion Wait
    BARO_READ_PRESSURE,    // Start pressure ADC Read
    BARO_CONV_TEMPERATURE, // Start Temp Conversion
    BARO_WAIT_TEMPERATURE, // Temp Conversion Wait
    BARO_READ_TEMPERATURE, // Start Temperature ADC Read
    BARO_READ_FINISH,      // Complete sensor reading
    BARO_READ_DONE,         // Done
    BARO_READ_FAIL
    } BARO_READ_STATE;
/* Global Variables ----------------------------------------------------------*/

/* Current Baro Sensor configuration */
static BARO_CONFIG   baro_configuration;

/* Baro calibration coefficients for measurement compensation */
static BARO_CAL_DATA_INT baro_cal_data;

/* Store pre-generated commands for sensor conversion
 * Derived from config, set during init
 */
static uint8_t pressure_cmd;
static uint8_t temperature_cmd;

/* Store pre-calculated sensor conversion timeouts
 * Derived from config, set during init, stored in microseconds
 */
static uint32_t pressure_timeout;
static uint32_t temperature_timeout;

/* Raw SPI bytes with readings */
uint8_t baro_raw_temp_buffer[3];
uint8_t baro_raw_press_buffer[3];

/* State of barometer read process */
static _Atomic BARO_READ_STATE baro_read_state = BARO_READ_FAIL;

static BARO_STATUS success;

/* Private function prototypes -----------------------------------------------*/

/* Some of these are probably good candidates for macros */

/*
 * Implementation Notes
 * I think we can create a TIM_OC_InitTypeDef, configure HAL_TIM_OC_ConfigChannel in a channel,
 * and then run HAL_TIM_OC_Start. Our callback is then HAL_TIM_OC_DelayElapsedCallback().
 */
static BARO_STATUS create_timer_interrupt
    (
    uint32_t timeout_us
    );

static BARO_STATUS clear_timer_interrupt
    (
    void
    );

static BARO_STATUS transmit_cmd_IT
    (
    uint8_t command_byte
    );

static BARO_STATUS transceive_adc_IT
    (
    uint8_t* out_buffer
    );

static void update_state
    (
    BARO_READ_STATE next_state
    );

static BARO_STATUS crc_verify
    (
    void
    );

/* Procedures ----------------------------------------------------------------*/

/* TODO
 * [X] write baro_init and its helpers
 * [X] write baro_get_IT (this is where the buffers will be converted to
 *   usable values and calculations applied to return the final float)
 * [X] write the timer interrupt creation functions
 * [ ] Make sure SPI NSS is done correctly
 * 
 */

/**
 * @brief Initialize barometric pressure sensor
 *
 * @details Load pre-calculated command values into their static variables,
 * calculate necessary sensor timeouts, and get calibration data.
 *
 * @retval Barometer status
 */
BARO_STATUS baro_init
    (
    BARO_CONFIG *config_ptr
    )
{
/* Run PROM verification */
BARO_STATUS crc_check = crc_verify();

if( crc_check == BARO_INVALID_PROM ) {
    return crc_check;
}

/* Setup the global configuration */
baro_configuration.enable |= config_ptr->enable;
baro_configuration.press_OSR_setting |= config_ptr->press_OSR_setting;
baro_configuration.temp_OSR_setting |= config_ptr->temp_OSR_setting;

/* Pre-calculate static command values */
switch(baro_configuration.press_OSR_setting)
    {
    case BARO_PRESS_OSR_X256:
        pressure_cmd = 0x40;
        pressure_timeout = 600;
        break;
    case BARO_PRESS_OSR_X512:
        pressure_cmd = 0x42;
        pressure_timeout = 1170;
        break;
    case BARO_PRESS_OSR_X1024:
        pressure_cmd = 0x44;
        pressure_timeout = 2280;
        break;
    case BARO_PRESS_OSR_X2048:
        pressure_cmd = 0x46;
        pressure_timeout = 4540;
        break;
    case BARO_PRESS_OSR_X4096:
        pressure_cmd = 0x48;
        pressure_timeout = 9040;
        break;
    default:
        /* An invalid enum value has been used */
        /* This should be unreachable under correct usage */
        debug_assert(false, ERROR_BARO_INIT_ERROR);
        return BARO_FAIL;
    }

switch(baro_configuration.temp_OSR_setting)
    {
    case BARO_TEMP_OSR_X256:
        temperature_cmd = 0x50;
        temperature_timeout = 600;
        break;
        case BARO_TEMP_OSR_X512:
        temperature_cmd = 0x52;
        temperature_timeout = 1170;
        break;
    case BARO_TEMP_OSR_X1024:
        temperature_cmd = 0x54;
        temperature_timeout = 2280;
        break;
    case BARO_TEMP_OSR_X2048:
        temperature_cmd = 0x56;
        temperature_timeout = 4540;
        break;
    case BARO_TEMP_OSR_X4096:
        temperature_cmd = 0x58;
        temperature_timeout = 9040;
        break;
    default:
        /* An invalid enum value has been used */
        /* This should be unreachable under correct usage */
        debug_assert(false, ERROR_BARO_INIT_ERROR);
        return BARO_FAIL;
    }

    /* Acquire calibration coefficients
     I loop through the PROM indicies of each calibration coefficient and save
     it to a buffer */
    uint16_t coeffs[6];

    HAL_StatusTypeDef read_success;

    for( int coeff = 1; coeff < 6; coeff += 1 ) {
        /* The first byte is the calculated PROM read command, and second is filler byte */
        uint8_t transmit_buffer[2] = {0xA0 + coeff << 1, 0xFF};
        uint8_t receive_buffer[2];

        /* Do the read */
        // TODO do more proper timeout
        read_success = HAL_SPI_TransmitReceive(BARO_SPI, transmit_buffer, receive_buffer, 2, 2000);

        if( read_success != HAL_OK ) {
            return BARO_FAIL; /* SPI communication failure */
        }

        /* Turn it into a usable format */
        coeffs[coeff - 1] = receive_buffer[0] << 8 + receive_buffer[1];
    }

    /* Set the calibration coefficients */
    baro_cal_data.par_c1 = coeffs[0];
    baro_cal_data.par_c2 = coeffs[1];
    baro_cal_data.par_c3 = coeffs[2];
    baro_cal_data.par_c4 = coeffs[3];
    baro_cal_data.par_c5 = coeffs[4];
    baro_cal_data.par_c6 = coeffs[5];

    return BARO_OK;
}

/**
  * @brief Check if barometer data is ready
  *
  * @details It looks at the barometer state machine status and checks if it
  * is in a successful terminal state (BARO_READ_DONE, not BARO_READ_FAIL).
  *
  * @retval Ready status
  */
bool baro_get_baro_data_ready
    (
    void
    )
{
return baro_read_state == BARO_READ_DONE;
}

/**
  * @brief Initiate reading of baro via interrupt mode
  *
  * @details If baro_read_state is in a terminal state (OK or FAIL), this resets
  * the state to the first state and calls the handler for the first time to
  * start the process. If it is NOT in a terminal state, that means a read is already
  * in progress, and this function will do nothing.
  *
  * @retval Status of the barometer
  */
BARO_STATUS baro_start_read_IT
    (
    void
    )
{
switch ( baro_read_state )
    {
    case BARO_READ_FAIL:

    case BARO_READ_DONE: // No currently running read
        baro_read_state = BARO_CONV_PRESSURE;
        baro_IT_handler(BARO_START_READ); // Keep FSM logic in that function
        break;

    default: // Read in progress
        return BARO_BUSY;
    }
}

/**
  * @brief Data acquisition finite state machine.
  *
  * @details This function must be called by three interrupt callbacks:
  * SPI_TxCpltCallback(), HAL_SPI_TxRxCpltCallback(), and
  * HAL_TIM_OC_DelayElapsedCallback(). These bring the read through a series of
  * 7 states (plus a done and fail state), needed to account for timeouts required by the chip.
  * You must pass in the corresponding event type for each one.
  *
  * @retval Status of the barometer
  */
BARO_STATUS baro_IT_handler
    (
    BARO_EVENT update_cause
    )
{
switch(baro_read_state) {
    // TODO actually deal with chip select pin
    case BARO_CONV_PRESSURE:
        if( update_cause != BARO_EVENT_START_READ )
            {
            /* Do nothing */
            return;
            }
        // 1. Called by baro_start_read_IT, start pressure conversion
        success = transmit_cmd_IT(pressure_cmd);
        update_state(BARO_WAIT_PRESSURE);
        break;
    case BARO_WAIT_PRESSURE:
        if( update_cause != BARO_EVENT_TX_CPLT )
            {
            /* Do nothing */
            return;
            }
        // 2. Called by SPI_TxCpltCallback(), create pressure conversion timeout
        success = create_timer_interrupt(pressure_timeout);
        update_state(BARO_READ_PRESSURE);
        break;
    case BARO_READ_PRESSURE:
        if( update_cause != BARO_EVENT_DELAY_ELAPSED )
            {
            /* Do nothing */
            return;
            }
        // 3. Called by HAL_TIM_OC_DelayElapsedCallback(), press temp ADC read
        clear_timer_interrupt();
        success = transceive_adc_IT(baro_raw_press_buffer);
        update_state(BARO_CONV_TEMPERATURE);
        break;
    case BARO_CONV_TEMPERATURE:
        if( update_cause != BARO_EVENT_TXRX_CPLT )
            {
            /* Do nothing */
            return;
            }
        // 4. Called by HAL_SPI_TxRxCpltCallback(), start temperature conversion
        success = transmit_cmd_IT(temperature_cmd);
        update_state(BARO_WAIT_TEMPERATURE);
        break;
    case BARO_WAIT_TEMPERATURE:
        if( update_cause != BARO_EVENT_TX_CPLT )
            {
            /* Do nothing */
            return;
            }
        // 5. Called by SPI_TxCpltCallback(), create temp conversion timeout
        success = create_timer_interrupt(temperature_timeout);
        update_state(BARO_READ_TEMPERATURE);
        break;
    case BARO_READ_TEMPERATURE:
        if( update_cause != BARO_EVENT_DELAY_ELAPSED )
            {
            /* Do nothing */
            return;
            }
        // 6. Called by HAL_TIM_OC_DelayElapsedCallback(), run temp ADC read
        clear_timer_interrupt();
        success = transceive_adc_IT(baro_raw_press_buffer);
        update_state(BARO_READ_FINISH);
        break;
    case BARO_READ_FINISH:
        if( update_cause != BARO_EVENT_TXRX_CPLT )
            {
            /* Do nothing */
            return;
            }
        // 7. Called by HAL_SPI_TxRxCpltCallback(), switch to done state.
        success = BARO_OK;
        update_state(BARO_READ_DONE);
        break;
    case BARO_READ_DONE:
        // We're done. Nothing happens.
        /* This state should never be entered during normal execution */
        debug_assert(true, ERROR_BARO_INVALID_STATE_ERROR);
        success = BARO_OK;
        break;
    default:
        /* This state should never be entered during normal execution */
        debug_assert(true, ERROR_BARO_INVALID_STATE_ERROR);
        success = BARO_FAIL; // BARO_READ_FAIL or undefined state
}
return success;
}

/* Helpers ----------------------------------------------------------------*/

/**
 * @brief Creates a timer interrupt for at least the specified duration
 *
 * @details This uses the microseconds timer, adding the requested time
 * to the current time in microseconds modulo the period, to avoid corner
 * cases with overflows
 *
 * @param timeout_us The time to wait in microseconds
 *
 * @retval The status of the barometer
 */
static BARO_STATUS create_timer_interrupt
    (
    uint32_t timeout_us
    )
{
    // This uses output capture/compare without changing pin
    // PSEUDOCODE
    // Create a TIM_OC_InitTypeDef
    // Set its pulse to __HAL_TIM_GET_COUNTER(&MICRO_TIM) + timeout_us
    // Set its mode to TIM_OCMODE_TIMING
    // etc...
    // Configure the given TIM channel with this.
    // Start the timer. Handlers are called in stm32h7xx_it.c

    /* Configure the output compare-based delay  */
    TIM_OC_InitTypeDef chip_delay;
    chip_delay.OCMode = TIM_OCMODE_TIMING; /* Disable any pin toglges */
    /* We do the modulo in order to consider the edge case that value is returned after timer wraparound */
    chip_delay.Pulse = (__HAL_TIM_GET_COUNTER(&MICRO_TIM) + timeout_us) % MICRO_TIM.Init.Period;
    
    /* I don't think these truly matter, as they mostly relate to physical pins */
    chip_delay.OCPolarity = TIM_OCPOLARITY_HIGH;
    chip_delay.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    chip_delay.OCFastMode = TIM_OCFAST_DISABLE;
    chip_delay.OCIdleState = TIM_OCIDLESTATE_RESET;

    /* Configure the TIM channel */
    HAL_StatusTypeDef timer_success = HAL_TIM_OC_ConfigChannel(&MICRO_TIM, &chip_delay, BARO_TIM_CHANNEL);

    if (timer_success != HAL_OK) {
        return BARO_FAIL;
    }
    
    /* Start the timer */
    timer_success = HAL_TIM_OC_Start_IT(&MICRO_TIM, BARO_TIM_CHANNEL);

    if(timer_success != HAL_OK) {
        return BARO_FAIL;
    }

    return BARO_OK;
}

/**
 * @brief Clears an existing timer interrupt
 *
 * @details Should be called after a timer event is received to disable the delay.
 * If this wasn't called, the output compare for the same event could theoretically be
 * fired multiple times (every timer period ellapse). In practice, the 71.5 minute
 * period of our microseconds timer makes this outcome unlikely, as the barometer
 * should have timed out well before then, but we do it as good practice.
 *
 * @retval The status of the barometer
 */
static BARO_STATUS clear_timer_interrupt
    (
    void
    )
{
HAL_StatusTypeDef timer_success = HAL_TIM_OC_Stop_IT(&MICRO_TIM, BARO_TIM_CHANNEL);

if (timer_success != HAL_OK) {
    return BARO_FAIL;
}

return BARO_OK;
}

/**
 * @brief Transmits a single command byte to the baro
 *
 * @param command_byte The command byte to transmit.
 *
 * @retval The status of the barometer
 */
static BARO_STATUS transmit_cmd_IT
    (
    uint8_t command_byte
    )
{
HAL_StatusTypeDef success;
success = HAL_SPI_Transmit_IT(BARO_SPI, &command_byte, 1);

if( success == HAL_OK ){
    return BARO_OK;
} else {
    return BARO_SPI_ERROR;
}
}

/**
 * @brief Requests the 3 bytes of data currently in the ADC
 *
 * @param out_buffer The 3 byte buffer to place received SPI bytes in.
 *
 * @retval The status of the barometer
 */
static BARO_STATUS transceive_adc_IT
    (
    uint8_t* out_buffer
    )
{
/* The zero byte is the ADC command, and the last two are filler bytes */
/* They should do nothing in the chip's command language */
/* This could still be wrong, though. We'll need to test on real HW. */
/* Each result is 24 bits (3 bytes) */
uint8_t in_buffer[3] = {0x00, 0xFF, 0xFF};

HAL_StatusTypeDef success;
success = HAL_SPI_TransmitReceive_IT(BARO_SPI, &in_buffer, &out_buffer, 3);

if( success == HAL_OK ){
    return BARO_OK;
} else {
    return BARO_SPI_ERROR;
}
}

/**
 * @brief Move to requested state if no fail conditions are detected
 *
 * @param next_state The baro read state to switch to.
 */
static void update_state
    (
    BARO_READ_STATE next_state
    )
{
if(success == BARO_OK)
    {
    baro_read_state = next_state;
    }
else
    {
    baro_read_state = BARO_READ_FAIL;
    }
}

/**
 * @brief Verify the intregrity of the barometer's internal PROM
 *
 * @retval PROM validity status
 */
static void crc_verify
    (
    void
    )
{
/* Read the PROM */
uint16_t prom[8];
for ( int addr = 0; addr < 8; addr++ ) {
    uint8_t transmit_buffer[2] = {0xA0 + addr << 1, 0xFF};
    uint8_t receive_buffer[2];

    HAL_SPI_TransmitReceive(BARO_SPI, transmit_buffer, receive_buffer, 2, 2000);

    /* Concatenate buffer result and put it into the prom array */
    prom[addr] = ( receive_buffer[0] << 8 ) + receive_buffer[1];
}

/* Go through the CRC verification process
 * This is heavily inspired by TE Connectivity/Measurement Specialites
 * Application Note 520, with changes made to better match SDR conventions.
 */
uint16_t remainder = 0x00;

/* The expected 4 bit CRC result */
uint16_t expected = prom[7] & 0x000F;

/* According to datasheet, the last byte must be zeroed out during
 * these calculations. */
prom[7] &= 0xFF00;

/* Loop through the bytes of the PROM */
for( int i = 0; i < 16; i++ ) {
	if( i % 2 == 1 ) { /* Select first or second byte of uint16_t */
		remainder ^= (prom[i >> 1]) & 0x00FF;
	} else {
		remainder ^= prom[i >> 1] >> 8;
	}
	for( int bit = 8; bit > 0; bit-- ) {
		if( remainder & 0x8000 ) {
			remainder = (remainder << 1) ^ 0x3000;
		} else {
			remainder = (remainder << 1);
		}
	}
}

/* Get final remainder */
remainder = (0x000F & (remainder >> 12)) ^ 0x00;

/* Compare remainder with expected to get return value */
if( remainder == expected ) {
    return BARO_OK;
} else {
    return BARO_INVALID_PROM;
}
}
