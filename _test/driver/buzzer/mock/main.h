/*******************************************************************************
*
* FILE: 
*      main.h (MOCK)
*
* DESCRIPTION: 
*      Header file to trick the test into compiling. Also contains mock function
*      definitions. For application-specific tests, use that app's main.h.
*
*******************************************************************************/

#include "pindefs.h"
#include "stm32h7xx_hal_uart.h"

void MOCK_HAL_Setup
    (
    HAL_StatusTypeDef status1ToReturn,
    HAL_StatusTypeDef status2ToReturn,
    HAL_StatusTypeDef status3ToReturn,
    HAL_StatusTypeDef status4ToReturn
    );
HAL_StatusTypeDef HAL_TIM_PWM_Start
    (
    TIM_HandleTypeDef* buzzTim,
    unsigned int buzzTimChannel
    );

HAL_StatusTypeDef HAL_TIM_PWM_Stop
    (
    TIM_HandleTypeDef* buzzTim,
    unsigned int buzzTimChannel
    );

void HAL_Delay
    (
    unsigned int timeToDelay
    );

