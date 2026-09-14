/*******************************************************************************
*
* FILE:
*      stm32h7xx_hal.h (unit test stub)
*
* DESCRIPTION:
*      Tiny HAL subset for onboard_flash unit tests on gcc host builds.
*
*******************************************************************************/

#ifndef STM32H7XX_HAL_STUB_H
#define STM32H7XX_HAL_STUB_H

#include <stdint.h>

#ifndef HAL_OK
#define HAL_OK      (0x00U)
#endif
#ifndef HAL_ERROR
#define HAL_ERROR   (0x01U)
#endif

typedef uint32_t HAL_StatusTypeDef;

/* Functions */
void __disable_irq
    (
    void
    );

void __enable_irq
    (
    void
    );

void __DSB
    (
    void
    );

void HAL_FLASH_Lock();
void HAL_FLASH_Unlock();

#endif /* STM32H7XX_HAL_STUB_H */

/*******************************************************************************
* END OF FILE
*******************************************************************************/
