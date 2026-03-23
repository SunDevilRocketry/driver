/*******************************************************************************
*
* FILE:
* 		timer.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_H 
#define TIMER_H 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

uint64_t get_us_tick
    (
    void
    );

void delay_ms
    (
    uint32_t delay
    );

void delay_us
    (
    uint32_t delay
    );

void micro_tim_IT_handler
    (
    void
    );

#ifdef __cplusplus
}
#endif
#endif /* TIMER_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
