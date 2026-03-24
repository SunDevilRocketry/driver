
/**
 * @file timer.c
 * 
 * @brief Timer delays and tick
 * 
 * @copyright
 *       Copyright (c) 2025 Sun Devil Rocketry.                                 
 *       All rights reserved.                                                   
 *                                                                              
 *       This software is licensed under terms that can be found in the LICENSE 
 *       file in the root directory of this software component.                 
 *       If no LICENSE file comes with this software, it is covered under the   
 *       BSD-3-Clause.                                                          
 *                                                                              
 *       https://opensource.org/license/bsd-3-clause        
 */

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal_tim.h"
#include "sdr_pin_defines_A0002.h"
#include "timer.h"


/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/

/* Increments every time the timer wraps around */
uint8_t micro_tim_wraparounds = 0;


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
 * @brief Getter for microsecond tick
 * 
 * Returns the microsecond tick as a sum of the number of wraparounds times the period plus the
 * current value in the counter register. This mitigates the fact that the timer itself 
 * wraps around approximately every 71.5 minutes if configured to count all the way to the 
 * uint32 max. This gives us over a million seconds or over 300 hours, which will probably be enough...
 * 
 * @return Microsecond tick
 */
uint64_t get_us_tick
    (
    void
    )
{
uint64_t tick = micro_tim_wraparounds * MICRO_TIM.Init.Period + __HAL_TIM_GET_COUNTER(&MICRO_TIM);
return tick;

} /* get_us_tick */


/**
 * @brief Minimum delay in miliseconds
 */
void delay_ms
    (
    uint32_t delay
    )
{
HAL_Delay(delay);

} /* delay_ms */


/**
 * @brief Minimum delay in microseconds
 */
void delay_us
    (
    uint32_t delay
    )
{
uint64_t start = get_us_tick();
uint64_t target = start + delay;

while ( get_us_tick() < target ) {}

} /* delay_us */


/**
 * @brief Upon timer overflow, increments the wraparound counter
 */
void micro_tim_IT_handler
    (
    void
    )
{
micro_tim_wraparounds++;

} /* micro_tim_IT_handler */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
