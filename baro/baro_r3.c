/**
  ******************************************************************************
  * @file           : baro_r3.c
  * @brief          : Driver for the barometer on FC rev 3.
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

/*
 * Implementation Notes
 * I think we can create a TIM_OC_InitTypeDef, configure HAL_TIM_OC_ConfigChannel in a channel,
 * and then run HAL_TIM_OC_Start. Our callback is then HAL_TIM_OC_DelayElapsedCallback().
 */

/* Includes ------------------------------------------------------------------*/

/* Standard */
#include <cstdint>
#include <stdbool.h>
#include <stdatomic.h>

/* Project */
#include "main.h"
#include "stm32h7xx_hal.h"
#include "sdr_pin_defines_A0010.h"
#include "baro.h"

/* Type Definitions ----------------------------------------------------------*/
typedef struct _BARO_CAL_DATA_INT
	{
	uint16_t par_c1; // Pressure Sensitity
	uint16_t par_c2; // Pressure Offset
	uint16_t par_c3; // Temp. Coeff. of Pressure Sensitivity
	uint16_t par_c4; // Temp. Coeff. of Pressure Offset
	uint16_t par_c5; // Reference Temperature
	uint16_t par_c6; // Temp. Coeff. of Temperature
	} BARO_CAL_DATA_INT;

/* Global Variables ----------------------------------------------------------*/

/* Current Baro Sensor configuration */
static BARO_CONFIG   baro_configuration;

/* Baro calibration coefficients for measurement compensation */
static BARO_CAL_DATA baro_cal_data;

static atomic_bool baro_data_ready = false;

