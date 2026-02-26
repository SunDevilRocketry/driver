/*******************************************************************************
*
* FILE:
* 		timer.c
*
* DESCRIPTION:
* 		Timer delays and tick
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


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal_tim.h"
#include "sdr_pin_defines_A0002.h"


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
* 		get_us_tick                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Getter for value in microsecond timer register                         *
* NOTE:                                                                        *
* 		This is a 32-bit timer that will wrap around after reaching            *
* 		the uint_32 max in approximately 71.5 minutes                          *
*                                                                              *
*******************************************************************************/
uint32_t get_us_tick
    (
    void
    )
{
return __HAL_TIM_GET_COUNTER(&MICRO_TIM);
} /* get_us_tick */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		delay_ms                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Minimum delay in miliseconds                                           *
*                                                                              *
*******************************************************************************/
void delay_ms
    (
    uint32_t delay
    )
{
HAL_Delay(delay);

} /* delay_ms */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		delay_us                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Minimum delay in microseconds                                          *
*                                                                              *
*******************************************************************************/
void delay_us
    (
    uint32_t delay
    )
{
uint32_t start = get_us_tick();
uint32_t target = start + delay;
if (target < start) /* timer will wrap around */
    {
    while ( get_us_tick() <= UINT32_MAX ) {}
    }

while ( get_us_tick() < target ) {}

} /* delay_us */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
