/**
  ******************************************************************************
  * @file    onboard_flash.c
  * @author  Sun Devil Rocketry
  * @brief   This file provides functions to program and erase the onboard
  *          flash on an STM32H750 microcontroller.
  * 
  * @note    Significant vibe-coded content. Should be tested *extensively*
  *          before being used in a production scenario.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE 
  * file in the root directory of this software component.                 
  * If no LICENSE file comes with this software, it is covered under the   
  * BSD-3-Clause.                                                          
  *                                                                              
  * https://opensource.org/license/bsd-3-clause     
  *
  ******************************************************************************
  */
 
/* Includes ------------------------------------------------------------------*/
#include <string.h>
 
#include "main.h"
#include "onboard_flash.h"
#include "common.h"
#include "error_sdr.h"
 
/* Compiler Protection -------------------------------------------------------*/
#ifndef STM32H750xx
_Static_assert( 0, "Unsupported hardware platform for onboard flash." );
#endif
 
/* Private variables ---------------------------------------------------------*/
 
/* Shadow buffer for full flash contents during erase-reprogram cycle.
 * Placed in RAM_D1 (512K) to avoid consuming DTCMRAM.
 * Must be 32-byte aligned to satisfy flashword write requirements. */
static uint8_t __attribute__((section(".ram_d1"), aligned(32), used))
    flash_shadow[ONBOARD_FLASH_SIZE];
 
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef onboard_flash_erase      (void);
static HAL_StatusTypeDef onboard_flash_write_word (uint32_t dst_addr, uint32_t* data);
static void              onboard_flash_wait_idle  (void);
static void              onboard_flash_caches_disable          (void);
static void              onboard_flash_caches_reset_and_enable (void);
 
/* Procedures ----------------------------------------------------------------*/
 
/**
 * Write data to the onboard flash at the given address.
 *
 * Performs a full shadow-erase-reprogram cycle:
 *   1. Copies entire flash to a RAM shadow buffer.
 *   2. Patches the shadow with the new data.
 *   3. Erases the single 128K flash sector.
 *   4. Reprograms flash from the shadow buffer.
 *
 * @param dst_addr  Starting flash address to write to. Must be 32-byte aligned
 *                  and fall within the flash region defined in the linker script.
 *                  Use the linker symbols (e.g. __user_config_start) directly.
 * @param data      Pointer to source data buffer.
 * @param len       Number of bytes to write. Must be a multiple of
 *                  ONBOARD_FLASH_WORD_SIZE (32 bytes).
 *
 * @note This function executes entirely from RAM. It is not safe to call from
 *       an ISR. Any terminal error triggers a panic via error_fail_fast().
 *
 * @return HAL_OK on success, HAL_ERROR if preconditions are not met.
 */
__attribute__((section(".RamFunc"), noinline))
HAL_StatusTypeDef onboard_flash_write_addr
    (
    uint32_t dst_addr,
    void* data,
    uint32_t len
    )
{
HAL_StatusTypeDef hal_status = HAL_OK;
uint32_t          offset;
uint32_t          word_addr;
 
/* Check preconditions */
if ( data == NULL )
    {
    return HAL_ERROR;
    }
if ( len % ONBOARD_FLASH_WORD_SIZE != 0 )   /* must be aligned to flashword  */
    {
    return HAL_ERROR;
    }
if ( dst_addr % ONBOARD_FLASH_WORD_SIZE != 0 )  /* dst must be flashword-aligned */
    {
    return HAL_ERROR;
    }
 
/* Copy entire flash contents into shadow buffer before the erase window.
 * memcpy executes from flash here — this is intentional and safe because
 * we have not yet entered the critical section or started any erase. */
memcpy( flash_shadow, (void *)FLASH_BASE, ONBOARD_FLASH_SIZE );
 
/* Patch the shadow buffer with the new data */
offset = dst_addr - FLASH_BASE;
memcpy( &flash_shadow[offset], data, len );
 
/* Disable caches before entering the critical section. Caches must be off
 * before erase/write to prevent stale ART lines from being fetched. */
onboard_flash_caches_disable();
 
/* Critical section: interrupts off for the duration of erase + reprogram.
 * Any ISR that fetches from flash after erase begins will hard fault. */
__disable_irq();
HAL_FLASH_Unlock();

/**
 * BEGIN CRITICAL SECTION
 */
 
hal_status |= onboard_flash_erase();
 
/* Reprogram flash word-by-word from the shadow buffer */
for ( uint32_t i = 0; i < ONBOARD_FLASH_SIZE; i += ONBOARD_FLASH_WORD_SIZE )
    {
    word_addr  = FLASH_BASE + i;
    hal_status |= onboard_flash_write_word( word_addr,
                                            (uint32_t *)&flash_shadow[i] );
    }
 
if ( hal_status != HAL_OK )
    {
    error_fail_fast( ERROR_FLASH_CMD_ERROR ); /* flash is corrupt -- panic */
    }
 
HAL_FLASH_Lock();
__enable_irq();

/**
 * END CRITICAL SECTION
 */
 
/* Reset and re-enable caches after programming. Caches must be reset here
 * to flush any lines that were cached before the erase. */
onboard_flash_caches_reset_and_enable();
 
return hal_status;
 
} /* onboard_flash_write_addr */
 
 
/**
 * Erases the entire onboard flash (bank 1, sector 0).
 *
 * @note Must be called from RAM with interrupts disabled and flash unlocked.
 *       Caller (onboard_flash_write_addr) is responsible for this.
 *
 * @return HAL_OK on success, HAL_ERROR if the flash controller reported errors.
 */
__attribute__((section(".RamFunc"), noinline))
static HAL_StatusTypeDef onboard_flash_erase
    (
    void
    )
{
uint32_t cr;
uint32_t sr;
 
onboard_flash_wait_idle();
 
/* Clear any pre-existing error flags before starting */
FLASH_CCR1 = FLASH_SR1_ERRORS;
 
/* Configure sector erase: bank 1, sector 0 */
cr  = FLASH_CR1;
cr &= ~( FLASH_CR1_PG | FLASH_CR1_BER | FLASH_CR1_SNB_Msk );
cr |=  FLASH_CR1_SER | ( 0UL << FLASH_CR1_SNB_Pos );
FLASH_CR1 = cr;
 
/* Trigger erase */
FLASH_CR1 |= FLASH_CR1_START;
 
/* Poll from RAM. No flash fetches occur here — the loop body only reads
 * the SR1 peripheral register at a fixed AHB address. */
onboard_flash_wait_idle();
 
/* Capture and clear error flags */
sr        = FLASH_SR1;
FLASH_CCR1 = FLASH_SR1_ERRORS;
 
/* Clear erase configuration bits */
FLASH_CR1 &= ~( FLASH_CR1_SER | FLASH_CR1_SNB_Msk );
 
return ( sr & FLASH_SR1_ERRORS ) ? HAL_ERROR : HAL_OK;
 
} /* onboard_flash_erase */
 
 
/**
 * Programs one 256-bit flashword (32 bytes) to flash.
 *
 * The H750 requires the minimum write unit to be a full 256-bit flashword.
 * Writing less than 32 bytes to a single flashword address is not supported
 * and will result in a programming error.
 *
 * @param dst_addr  Destination flash address. Must be 32-byte aligned.
 * @param data      Pointer to 32 bytes (8 x uint32_t) of source data.
 *                  Must also be 32-byte aligned.
 *
 * @note Must be called from RAM with interrupts disabled and flash unlocked.
 *       Caller (onboard_flash_write_addr) is responsible for this.
 *
 * @return HAL_OK on success, HAL_ERROR if the flash controller reported errors.
 */
__attribute__((section(".RamFunc"), noinline))
static HAL_StatusTypeDef onboard_flash_write_word
    (
    uint32_t  dst_addr,
    uint32_t* data
    )
{
uint32_t          sr;
volatile uint32_t *dst = (volatile uint32_t *)dst_addr;
 
onboard_flash_wait_idle();
 
/* Clear any pre-existing error flags before starting */
FLASH_CCR1 = FLASH_SR1_ERRORS;
 
/* Enable programming. PSIZE=11 selects 64-bit write parallelism,
 * required when VCC is in the 2.7V-3.6V range (RM0433 §4.3.10). */
FLASH_CR1 = ( FLASH_CR1 & ~( FLASH_CR1_SER | FLASH_CR1_BER ) )
            | FLASH_CR1_PG
            | FLASH_CR1_PSIZE_32;
 
/* Write 8 x 32-bit words (256 bits) sequentially.
 * All 8 writes must be to the same flashword address range and must
 * complete before the controller triggers the internal program operation. */
dst[0] = data[0];
dst[1] = data[1];
dst[2] = data[2];
dst[3] = data[3];
dst[4] = data[4];
dst[5] = data[5];
dst[6] = data[6];
dst[7] = data[7];
 
/* DSB ensures all buffered writes retire to the flash write buffer
 * before we poll BSY, preventing a false-idle read. */
__DSB();
 
onboard_flash_wait_idle();
 
/* Capture and clear error flags */
sr         = FLASH_SR1;
FLASH_CCR1 = FLASH_SR1_ERRORS;
 
/* Clear PG to end programming mode */
FLASH_CR1 &= ~FLASH_CR1_PG;
 
return ( sr & FLASH_SR1_ERRORS ) ? HAL_ERROR : HAL_OK;
 
} /* onboard_flash_write_word */
 
 
/**
 * Waits for the flash controller to become idle.
 *
 * Polls until BSY, WBNE, and QW are all clear:
 *   BSY  — an operation is actively executing
 *   WBNE — the write buffer has not yet been flushed to the array
 *   QW   — an operation is queued but not yet started
 *
 * @note Must be called from RAM. The polling loop contains no flash fetches.
 */
__attribute__((section(".RamFunc"), noinline))
static void onboard_flash_wait_idle
    (
    void
    )
{
while ( FLASH_SR1 & ( FLASH_SR1_BSY | FLASH_SR1_WBNE | FLASH_SR1_QW ) );
 
} /* onboard_flash_wait_idle */
 
 
/**
 * Disables the instruction and data caches.
 *
 * Must be called before any erase or program operation. Leaving caches
 * enabled during flash modification risks the CPU fetching stale ART lines.
 *
 * @note Does not need to run from RAM — called before the critical section.
 */
static void onboard_flash_caches_disable
    (
    void
    )
{
FLASH_ACR &= ~( FLASH_ACR_ICEN | FLASH_ACR_DCEN );
 
} /* onboard_flash_caches_disable */
 
 
/**
 * Resets and re-enables the instruction and data caches.
 *
 * Must be called after flash programming is complete. The reset step
 * invalidates any lines that were cached before the erase, preventing
 * the CPU from executing stale content on the next fetch.
 *
 * @note Does not need to run from RAM — called after the critical section.
 */
static void onboard_flash_caches_reset_and_enable
    (
    void
    )
{
/* Assert reset, then deassert, then re-enable */
FLASH_ACR |=  ( FLASH_ACR_ICRST | FLASH_ACR_DCRST );
FLASH_ACR &= ~( FLASH_ACR_ICRST | FLASH_ACR_DCRST );
FLASH_ACR |=  ( FLASH_ACR_ICEN  | FLASH_ACR_DCEN  );
 
} /* onboard_flash_caches_reset_and_enable */