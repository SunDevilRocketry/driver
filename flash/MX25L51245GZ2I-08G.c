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
#include "sdr_pin_defines_A0010.h"

/* Global Variables ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/

static FLASH_STATUS flash_qspi_enable
    (
    void
    );

static FLASH_STATUS flash_write_enable
    (
    void
    );

static FLASH_STATUS flash_enable_4byte_addressing
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


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status    = FLASH_OK;
pflash_handle -> status_register = 0;

/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Activate QSPI */
if ( flash_qspi_enable() != FLASH_OK )
    {
    return FLASH_INIT_FAIL;
    }

/* Write enable */
if ( flash_write_enable() != FLASH_OK )
    {
    return FLASH_INIT_FAIL;
    }
else
    {
    pflash_handle -> write_protected = false;
    }

/* Enable 4-byte addressing */
if ( flash_enable_4byte_addressing() != FLASH_OK )
    {
    return FLASH_INIT_FAIL;
    }

/* Check the flash chip status register to confirm chip can be reached */
flash_status = flash_get_status( pflash_handle );
if ( flash_status != FLASH_OK )
    {
    return flash_status;
    }
else if ( !(pflash_handle -> status_register & FLASH_STATUS_REG_QUAD_ENABLED) )
    {
    return FLASH_INIT_FAIL;
    }

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
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_NONE;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;

/* Transmit (blocking) */
hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, HAL_FLASH_TIMEOUT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_qspi_enable */


static FLASH_STATUS flash_write_enable
    (
    void
    )
{
/* Initializations */
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};

/* Construct write enable command (no address/data) */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_WRITE_ENABLE_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_NONE;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

/* Transmit (blocking) */
hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, HAL_FLASH_TIMEOUT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_write_enable */


FLASH_STATUS flash_get_status
    (
    HFLASH_BUFFER* pflash_handle
    )
{
/* Initializations */
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};

if ( pflash_handle == NULL )
    {
    return FLASH_INVALID_INPUT;
    }

/* Construct "Read Status Register" command */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_READ_STATUS_REG_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_1_LINE;
spi_command.NbData = 1;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, HAL_FLASH_TIMEOUT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

hal_status = HAL_OSPI_Receive(&FLASH_OSPI, &(pflash_handle->status_register), HAL_FLASH_TIMEOUT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

return FLASH_OK;

} /* flash_get_status */


static FLASH_STATUS flash_enable_4byte_addressing
    (
    void
    )
{
/* Initializations */
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};

/* Construct "Enter 4-byte address mode" command (no address/data) */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_ENABLE_4BYTE_ADDR_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_NONE;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, HAL_FLASH_TIMEOUT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_enable_4byte_addressing */