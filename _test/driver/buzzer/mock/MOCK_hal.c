/**
  ********************************************************************************
  * @file MOCK_hal.c
  * @brief Mock HAL functions for the buzzer driver unit tests
  *******************************************************************************
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
  *
  ******************************************************************************/
#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef mocked_returns[] = { HAL_OK, HAL_OK, HAL_OK, HAL_OK }; /* Default to "OK" return */
int cur_return = 0;
int curTestType = 0;
int delayCallCount = 0;
extern int delayCalls[5];

extern bool HAL_TIM_PWM_Start_called_correctly;
extern bool HAL_TIM_PWM_Stop_called_correctly;



/* Used to mock these functions */
void MOCK_HAL_Setup
    (
    HAL_StatusTypeDef status1ToReturn,
    HAL_StatusTypeDef status2ToReturn,
    HAL_StatusTypeDef status3ToReturn,
    HAL_StatusTypeDef status4ToReturn
    )
{

mocked_returns[0] = status1ToReturn;
mocked_returns[1] = status2ToReturn;
mocked_returns[2] = status3ToReturn;
mocked_returns[3] = status4ToReturn;


cur_return = 0;
delayCallCount = 0;
} /* MOCK_HAL_Setup */

/* Reset mock HAL setup */
void MOCK_HAL_Reset
    (
        void
    )
{
HAL_TIM_PWM_Start_called_correctly = false;
HAL_TIM_PWM_Stop_called_correctly = false;
for ( int i = 0; i<5; i++ )
	{
	delayCalls[i] = -1;
	}
for ( int i = 0; i<4; i++ )
    {
    mocked_returns[i] = HAL_OK;
    }

} /* MOCK_HAL_Reset */

HAL_StatusTypeDef HAL_TIM_PWM_Start
    (
    TIM_HandleTypeDef* buzzTim,
    unsigned int buzzTimChannel
    )
{
if ( buzzTim == &BUZZ_TIM && buzzTimChannel == BUZZ_TIM_CHANNEL )
    {
    HAL_TIM_PWM_Start_called_correctly = true;
    } 

return mocked_returns[cur_return++];
} /* HAL_TIM_PWM_Start */


HAL_StatusTypeDef HAL_TIM_PWM_Stop
    (
    TIM_HandleTypeDef* buzzTim,
    unsigned int buzzTimChannel
    )
{
if ( buzzTim == &BUZZ_TIM && buzzTimChannel == BUZZ_TIM_CHANNEL )
    {
    HAL_TIM_PWM_Stop_called_correctly = true;
    }

return mocked_returns[cur_return++];
} /* HAL_TIM_PWM_Stop */


void HAL_Delay
    (
    unsigned int timeToDelay
    )
{   
delayCalls[delayCallCount] = timeToDelay;
delayCallCount++;
} /* HAL_Delay */

