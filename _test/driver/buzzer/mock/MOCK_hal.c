/*******************************************************************************
*
* FILE: 
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for HAL to trick
*      tests into compiling.
*
*******************************************************************************/

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef mocked_returns[] = { HAL_OK, HAL_OK }; /* Default to "OK" return */
int cur_return = 0;

extern bool HAL_TIM_PWM_Start_called_correctly;
extern bool HAL_TIM_PWM_Stop_called_correctly;
extern bool HAL_Delay_called_correctly;

/* Used to mock these functions */
void MOCK_HAL_Status_Returns
    (
    HAL_StatusTypeDef status1ToReturn,
    HAL_StatusTypeDef status2ToReturn
    )
{
mocked_returns[0] = status1ToReturn;
mocked_returns[1] = status2ToReturn;

cur_return = 0;
}

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
}

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
}

void HAL_Delay
    (
    unsigned int timeToDelay
    )
{
    if ( timeToDelay == 500)
    {
        HAL_Delay_called_correctly = true;
    }

}

