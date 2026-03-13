/**
  ******************************************************************************
  * @file           : MX25L51245GZ2I-08G.c
  * @brief          : Driver for the flash chip on FC rev 3.
  ******************************************************************************
  * @attention
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
  ******************************************************************************
  @verbatim
  ==============================================================================
                      ##### Flash driver features #####
  ==============================================================================
  [..]
  (+) Implements the same APIs as the legacy driver
  ******************************************************************************
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "flash.h"

#include "stm32h7xx_hal.h"


/* Global Variables ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

static FLASH_STATUS flash_qspi_enable
    (
    void
    );


/* Procedures ----------------------------------------------------------------*/

/**
  * @brief Initializes the flash driver
  *
  * @param pflash_handle Pointer to the flash handle for this device.
  * @retval The status of the flash peripheral.
  */
FLASH_STATUS flash_init 
	(
	HFLASH_BUFFER* pflash_handle
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS flash_status;    /* Flash API function return codes        */
uint8_t      status_register; /* Desired status register contents       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status    = FLASH_OK;
status_register = 0U;


/*------------------------------------------------------------------------------
 Pre-processing 
------------------------------------------------------------------------------*/

// /* Check for invalid BPL setting */
// if ( pflash_handle -> bpl_bits > FLASH_BPL_ALL )
// 	{
// 	return FLASH_INVALID_INPUT; 
// 	}

// /* Determine the desired status register contents */
// status_register &= pflash_handle -> bpl_bits;
// status_register |= pflash_handle -> bpl_write_protect;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Activate QSPI */
if ( flash_qspi_enable() != FLASH_OK )
    {
    return FLASH_INIT_FAIL;
    }

/* Write enable */
// if ( pflash_handle -> write_protected )
// 	{
// 	flash_write_disable();
// 	return FLASH_OK;
// 	}
// else
// 	{
// 	flash_write_enable();
// 	}

// /* Check the flash chip status register to confirm chip can be reached */
// flash_status = flash_get_status( pflash_handle );
// if ( flash_status != FLASH_OK )
// 	{
// 	return flash_status;
// 	}
// else if ( pflash_handle -> status_register == 0xFF )
// 	{
// 	return FLASH_INIT_FAIL;
// 	}

// /* Disable writing to the chip in case of prior interrupted write operation */
// if ( write_disable() != FLASH_OK )
// 	{
// 	return FLASH_CANNOT_WRITE_DISABLE;
// 	}

// /* Set the bpl bits in the flash chip */
// flash_status = flash_set_status( status_register );

// /* Confirm Status register contents */
// while( flash_is_flash_busy() == FLASH_BUSY ){}
// flash_status = flash_get_status( pflash_handle );
// if ( pflash_handle -> status_register != status_register )
//     {
//     return FLASH_INIT_FAIL;
//     }
// else
//     {
//     return flash_status;
//     }

return FLASH_OK;

} /* flash_init */


static FLASH_STATUS flash_qspi_enable
    (
    void
    )
{
/* Initializations */
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};

/* Construct QSPI enable command (single SPI mode) */
spi_command.Instruction = FLASH_ENABLE_QSPI_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;

/* Transmit (blocking) */
hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

return ( hal_status == HAL_OK );

} /* flash_qspi_enable */