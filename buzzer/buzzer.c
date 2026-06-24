/*******************************************************************************
*
* FILE: 
* 		buzzer.c
*
* DESCRIPTION: 
* 		Contains API functions for the flight computer buzzer 
*
* COPYRIGHT:                                                                   
*       Copyright (c) 2025 Sun Devil Rocketry.                                 
*       All rights reserved.                                                   
*                                                                              
*       This software is licensed under terms that can be found in the LICENSE 
*       file in the root directory of this software component.                 
*       If no LICENSE file comes with this software, it is covered under the   
*       BSD-3-Clause.                                                          
*                                                                              
*       https://opensource.org/license/bsd-3-clause          
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#if defined( A0002_REV2 )
	#include "sdr_pin_defines_A0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#elif defined ( A0010 )
	#include "sdr_pin_defines_A0010.h"
#else
	#error No buzzer compatible device specified
#endif

#ifdef EMULATOR
#include "emulator.h"
#endif

#include "buzzer.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       buzzer_beep                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Beep the flight computer buzzer                                        *
*                                                                              *
*******************************************************************************/
BUZZ_STATUS buzzer_beep 
	(
	uint32_t duration /* Length of beep in milliseconds */
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status; /* Return codes from HAL API */

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status = HAL_OK;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/
#ifndef EMULATOR
/* Start generating PWM pulses */
hal_status = HAL_TIM_PWM_Start( &(BUZZ_TIM), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}

/* Wait for beep duration */
HAL_Delay( duration );

/* Stop the PWM pulses */
hal_status = HAL_TIM_PWM_Stop( &( BUZZ_TIM ), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}
else
	{
	return BUZZ_OK;
	}
#else
emulator_buzzer_beep_request(duration);
return hal_status;
#endif

} /* buzzer_beep */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       buzzer_multi_beeps                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Beep the flight computer buzzer a specified number of times (blocking) *
*                                                                              *
*******************************************************************************/
BUZZ_STATUS buzzer_multi_beeps
	(
	uint32_t beep_duration, 		/* Length of beep in milliseconds */
	uint32_t time_between_beeps,	/* How long to wait between beeps in ms */
	uint8_t	 num_beeps 				/* How many times to repeat */
	)
{
BUZZ_STATUS status = BUZZ_OK;
for ( uint8_t i = 0; i < num_beeps; i++ )
	{
	status = buzzer_beep(beep_duration);

	if ( status != BUZZ_OK )
		{
		return status;
		}
	
	HAL_Delay(time_between_beeps);
	}

	return status;

}


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/