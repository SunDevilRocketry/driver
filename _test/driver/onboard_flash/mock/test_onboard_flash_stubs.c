/*******************************************************************************
*
* FILE:
*      test_onboard_flash_stubs.c
*
* DESCRIPTION:
*      Stubs for symbols referenced by onboard_flash_write_addr when linked for
*      host unit tests (HAL flash, CMSIS-style intrinsics, error_fail_fast).
*
*******************************************************************************/

#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "error_sdr.h"

void HAL_FLASH_Unlock
    (
    void
    )
{
}

void HAL_FLASH_Lock
    (
    void
    )
{
}

void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
    (void)error_code;
}

/* ARM CMSIS intrinsics are not available on x86_64 host gcc */
void __disable_irq
    (
    void
    )
{
}

void __enable_irq
    (
    void
    )
{
}

void __DSB
    (
    void
    )
{
}

/*******************************************************************************
* END OF FILE
*******************************************************************************/
