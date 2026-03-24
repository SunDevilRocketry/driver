/**
  ******************************************************************************
  * @file    onboard_flash.h
  * @author  Sun Devil Rocketry
  * @brief   This file provides functions to program and erase the onboard
  *          flash on an STM32H750 microcontroller.
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
 
#ifndef ONBOARD_FLASH_H
#define ONBOARD_FLASH_H
 
#ifdef __cplusplus
extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
 
/* Defines -------------------------------------------------------------------*/
 
/* Flash register block base address, RM0433 Table 8 */
#define FLASH_BASE_REG      0x52002000UL
 
#define FLASH_ACR           (*(volatile uint32_t *)(FLASH_BASE_REG + 0x000))
#define FLASH_KEYR1         (*(volatile uint32_t *)(FLASH_BASE_REG + 0x004))
#define FLASH_CR1           (*(volatile uint32_t *)(FLASH_BASE_REG + 0x00C))
#define FLASH_SR1           (*(volatile uint32_t *)(FLASH_BASE_REG + 0x010))
#define FLASH_CCR1          (*(volatile uint32_t *)(FLASH_BASE_REG + 0x014))
 
/* CR1 bits */
#define FLASH_CR1_LOCK      (1UL << 0)
#define FLASH_CR1_PG        (1UL << 1)
#define FLASH_CR1_SER       (1UL << 2)
#define FLASH_CR1_BER       (1UL << 3)
#define FLASH_CR1_START     (1UL << 7)
#define FLASH_CR1_PSIZE_Pos 4
#define FLASH_CR1_PSIZE_32  (0x3UL << FLASH_CR1_PSIZE_Pos)  /* VCC 2.7-3.6V  */
#define FLASH_CR1_SNB_Pos   8
#define FLASH_CR1_SNB_Msk   (0x7UL << FLASH_CR1_SNB_Pos)
 
/* SR1 bits */
#define FLASH_SR1_BSY       (1UL << 0)   /* operation actively running        */
#define FLASH_SR1_WBNE      (1UL << 1)   /* write buffer not empty            */
#define FLASH_SR1_QW        (1UL << 2)   /* wait queue not empty              */
#define FLASH_SR1_ERRORS    (0x03EEF800UL) /* all error bits, RM0433 §4.9.5   */
 
/* ACR bits (cache control) */
#define FLASH_ACR_ICEN      (1UL << 9)
#define FLASH_ACR_DCEN      (1UL << 10)
#define FLASH_ACR_ICRST     (1UL << 11)
#define FLASH_ACR_DCRST     (1UL << 12)
 
/* H750 has one 128K sector. Total flash and flashword sizes in bytes.
 * A flashword is the minimum programmable unit on the H750 (256 bits). */
#define ONBOARD_FLASH_SIZE          (128u * 1024u)
#define ONBOARD_FLASH_WORD_SIZE     (32u)
 
/* Public function prototypes ------------------------------------------------*/
 
HAL_StatusTypeDef onboard_flash_write_addr(uint32_t dst_addr,
                                            uint8_t* data,
                                            uint32_t len);
 
#ifdef __cplusplus
}
#endif
 
#endif /* ONBOARD_FLASH_H */