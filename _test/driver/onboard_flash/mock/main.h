/*******************************************************************************
*
* FILE:
*      main.h (unit test stub for onboard_flash)
*
* DESCRIPTION:
*      Minimal definitions so driver/onboard_flash/onboard_flash.c compiles on
*      the host. Application tests should use the target app's main.h instead.
*
*******************************************************************************/

#ifndef TEST_ONBOARD_FLASH_MAIN_H
#define TEST_ONBOARD_FLASH_MAIN_H

#include <stdint.h>

/* CMSIS FLASH_BANK1_BASE is 0x08000000 on H7; only used if a test reaches
 * memcpy from flash (precondition tests return before that). */
#ifndef FLASH_BASE
#define FLASH_BASE (0x08000000UL)
#endif

#include "stm32h7xx_hal.h"

#endif /* TEST_ONBOARD_FLASH_MAIN_H */

/*******************************************************************************
* END OF FILE
*******************************************************************************/
