/*******************************************************************************
*
* FILE: 
* 		led.c
*
* DESCRIPTION: 
* 		Contains API functions to set the behavior of the on-board rgb led
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
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( A0002_REV2      		) /* FC rev2 */
	#include "sdr_pin_defines_A0002.h"
#elif defined( A0010 		   		) /* FC rev3 */
	#include "sdr_pin_defines_A0010.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( GROUND_STATION       )
	#include "sdr_pin_defines_A0005.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#elif defined( VALVE_CONTROLLER     )
	#include "sdr_pin_defines_L0005.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "led.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		led_reset                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Resets the RGB led                                                     *
*                                                                              *
*******************************************************************************/
void led_reset
	(
	void
	)
{

/* Set all MCU RGB led pins to high */
HAL_GPIO_WritePin(
                 STATUS_GPIO_PORT, 
                 STATUS_R_PIN |
                 STATUS_G_PIN |
                 STATUS_B_PIN,
                 GPIO_PIN_SET
                 );

} /* led_reset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		led_set_color                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Sets the LED to a color from the LED_COLOR_CODES enum                  *
*                                                                              *
*******************************************************************************/
void led_set_color
	(
	LED_COLOR_CODES color
	)
{

/* Reset LED */
#ifndef EMULATOR
led_reset(); /* The call to reset every time makes the emulator GUI flash */
#endif

/* Check Colors */
switch ( color )
	{
	case LED_GREEN:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, STATUS_G_PIN, GPIO_PIN_RESET );
		break;
		}

	case LED_RED:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, STATUS_R_PIN, GPIO_PIN_RESET );
		break;
		}

	case LED_BLUE:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, STATUS_B_PIN, GPIO_PIN_RESET );
		break;
		}

	case LED_CYAN:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, 
                           STATUS_B_PIN | 
                           STATUS_G_PIN, 
                           GPIO_PIN_RESET);
		break;
		}

	case LED_PURPLE:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, 
		                   STATUS_B_PIN | 
						   STATUS_R_PIN    ,
						   GPIO_PIN_RESET );
		break;
		}
	
	case LED_YELLOW:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, 
		                   STATUS_G_PIN | 
						   STATUS_R_PIN    ,
						   GPIO_PIN_RESET );
		break;
		}

	case LED_WHITE:
		{
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, 
		                   STATUS_G_PIN | 
						   STATUS_R_PIN |
						   STATUS_B_PIN   ,
						   GPIO_PIN_RESET );
		break;
		}
	} /* switch ( color ) */
} /* led_set_color */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/