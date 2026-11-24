/**
  ********************************************************************************
  * @file main.h (MOCK)
  * @brief Header file to trick the test into compiling. Also contains mock function definitions. For application-specific tests, use that app's main.h.
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

