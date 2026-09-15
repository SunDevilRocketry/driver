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
#include "onboard_flash.h"

static uint32_t injected_failure_word = UINT32_MAX;
static uint32_t written_word_count;
ERROR_CODE      last_error_code;

void onboard_flash_test_fail_on_word
    (
    uint32_t word_number
    )
{
injected_failure_word = word_number;
written_word_count = 0;
}

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
last_error_code = error_code;
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
if ( written_word_count == injected_failure_word )
    {
    FLASH_SR1 |= FLASH_SR1_ERRORS;
    }

written_word_count++;
}

/*******************************************************************************
* END OF FILE
*******************************************************************************/
