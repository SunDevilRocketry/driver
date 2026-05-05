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

static FLASH_STATUS flash_wait_ready
    (
    uint32_t timeout_ms
    );


/* Procedures ----------------------------------------------------------------*/

/**
  * @brief Initializes the flash driver
  * 
  * @details This function should be called before any other in the flash
  * driver, as it initializes the driver to a known state that allows other
  * commands to function properly.
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


/**
 * @brief Erase the entire contents of the flash chip.
 * 
 * @param pflash_handle A pointer to the flash handle object to be used for the operation.
 * @retval The status of the flash peripheral.
 */
FLASH_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle
    )
{
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};

if ( pflash_handle == NULL )
    {
    return FLASH_INVALID_INPUT;
    }
else if ( pflash_handle -> write_protected )
    {
    return FLASH_WRITE_PROTECTED;
    }

/* Construct command */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_CHIP_ERASE_CMD;
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

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

/* Wait until the device clears BUSY bit */
return flash_wait_ready(FLASH_TIMEOUT_ERASE);

} /* flash_erase */


/**
 * @brief Erase a block of flash.
 * 
 * @param flash_block_num The number of the block to erase (0-indexed).
 * @param size The size of the block to erase.
 * 
 * @retval The status of the flash peripheral.
 */
FLASH_STATUS flash_block_erase
    (
    FLASH_BLOCK      flash_block_num,
    FLASH_BLOCK_SIZE size
    )
{
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};
uint32_t addr = 0;
uint32_t end_addr = 0;
uint32_t opcode = 0;

/* Convert the block number/size to addresses and operations */
switch ( size )
    {
    case FLASH_BLOCK_4K:
        opcode = FLASH_SECTOR_ERASE_4KB_CMD;
        addr   = ((uint32_t)flash_block_num) * 0x1000u;
        end_addr = (addr + 0x1000) - 1;
        break;

    case FLASH_BLOCK_32K:
        opcode = FLASH_BLOCK_ERASE_32KB_CMD;
        addr   = ((uint32_t)flash_block_num) * 0x8000u;
        end_addr = (addr + 0x8000) - 1;
        break;

    case FLASH_BLOCK_64K:
        opcode = FLASH_BLOCK_ERASE_64KB_CMD;
        addr   = ((uint32_t)flash_block_num) * 0x10000u;
        end_addr = (addr + 0x10000) - 1;
        break;

    default:
        return FLASH_INVALID_INPUT;
    }

/* Validate addresses */
if( end_addr > FLASH_MAX_ADDR )
    {
    return FLASH_INVALID_INPUT;
    }

/* Construct command */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = opcode;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.Address = addr;
spi_command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
spi_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_NONE;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

return flash_wait_ready(FLASH_TIMEOUT_ERASE);

} /* flash_block_erase */


/**
 * @brief Write up to a page (256 bytes) to the flash memory.
 * 
 * @param pflash_handle A pointer to the flash handle object to be used for the operation.
 * @retval The status of the flash peripheral.
 */
FLASH_STATUS flash_write
    (
    HFLASH_BUFFER* pflash_handle
    )
{
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};
size_t len = 0;

if ( (pflash_handle == NULL) || (pflash_handle->pbuffer == NULL) )
    {
    return FLASH_INVALID_INPUT;
    }

len = pflash_handle->num_bytes;
if ( ( len == 0u ) 
  || ( len > FLASH_PAGE_SIZE ) )
    {
    return FLASH_INVALID_INPUT;
    }

/* Validate addresses */
if( ( pflash_handle->address + len ) - 1 > FLASH_MAX_ADDR )
    {
    return FLASH_INVALID_INPUT;
    }

/* Assumes flash_init() already enabled QSPI + write enable */

spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_PAGE_PROGRAM_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.Address = pflash_handle->address;
spi_command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
spi_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_4_LINES;
spi_command.NbData = len;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = 0;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

hal_status = HAL_OSPI_Transmit(&FLASH_OSPI, pflash_handle->pbuffer, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

return flash_wait_ready(FLASH_TIMEOUT_DEFAULT);

} /* flash_write */


/**
 * @brief Read data from the flash memory.
 * 
 * @param pflash_handle A pointer to the flash handle object to be used for the operation.
 * @retval The status of the flash peripheral.
 */
FLASH_STATUS flash_read
    (
    HFLASH_BUFFER* pflash_handle,
    uint32_t       num_bytes
    )
{
HAL_StatusTypeDef hal_status = HAL_OK;
OSPI_RegularCmdTypeDef spi_command = {0};
size_t len = 0;

/* Verify valid pointers */
if ( ( pflash_handle == NULL ) 
  || ( pflash_handle->pbuffer == NULL ) )
    {
    return FLASH_INVALID_INPUT;
    }

/* If we're reading more bytes than the buffer can handle, we fail so we prevent overflow */
len = num_bytes;
if ( ( len == 0u ) 
  || ( len > pflash_handle->num_bytes ) )
    {
    return FLASH_INVALID_INPUT;
    }

/* Validate addresses */
if( ( pflash_handle->address + len ) - 1 > FLASH_MAX_ADDR )
    {
    return FLASH_INVALID_INPUT;
    }

/* Construct command */
spi_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
spi_command.FlashId = HAL_OSPI_FLASH_ID_1;
spi_command.Instruction = FLASH_READ_CMD;
spi_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
spi_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
spi_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
spi_command.Address = pflash_handle->address;
spi_command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
spi_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
spi_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
spi_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
spi_command.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
spi_command.DataMode = HAL_OSPI_DATA_4_LINES;
spi_command.NbData = len;
spi_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
spi_command.DummyCycles = FLASH_READ_DUMMY_CYCLES;
spi_command.DQSMode = HAL_OSPI_DQS_DISABLE;
spi_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

hal_status = HAL_OSPI_Receive(&FLASH_OSPI, pflash_handle->pbuffer, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

return FLASH_OK;

} /* flash_read */


/**
 * @brief Enable QSPI mode on the flash peripheral.
 * 
 * @retval The status of the flash peripheral.
 */
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
hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_qspi_enable */


/**
 * @brief Enable writes to the flash peripheral.
 * 
 * @retval The status of the flash peripheral.
 */
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
hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_write_enable */


/**
 * @brief Retrieve the status register on the flash peripheral.
 * 
 * @param pflash_handle A pointer to the flash handle object to be used for the operation.
 * @retval The status of the flash peripheral.
 */
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

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

hal_status = HAL_OSPI_Receive(&FLASH_OSPI, &(pflash_handle->status_register), FLASH_TIMEOUT_DEFAULT);
if ( hal_status != HAL_OK )
    {
    return FLASH_FAIL;
    }

return FLASH_OK;

} /* flash_get_status */


/**
 * @brief Enable four-byte addresses on the flash peripheral.
 * 
 * @retval The status of the flash peripheral.
 */
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

hal_status = HAL_OSPI_Command(&FLASH_OSPI, &spi_command, FLASH_TIMEOUT_DEFAULT);

if( hal_status == HAL_OK )
    {
    return FLASH_OK;
    }
else
    {
    return FLASH_FAIL;
    }

} /* flash_enable_4byte_addressing */


/**
 * @brief Busy-wait until the flash signals ready status.
 * 
 * @param timeout_ms The amount of time to wait (in ms) for the operation to complete.
 * @retval The status of the flash peripheral.
 */
static FLASH_STATUS flash_wait_ready
    (
    uint32_t timeout_ms
    )
{
uint32_t start = HAL_GetTick();
HFLASH_BUFFER handle = {0};

while ( true )
    {
    if ( flash_get_status( &handle ) != FLASH_OK )
        {
        return FLASH_SPI_ERROR;
        }

    if ( ( handle.status_register & FLASH_STATUS_REG_WIP ) == 0u )
        {
        return FLASH_OK;
        }

    if ( timeout_ms != 0xFFFFFFFFu )
        {
        if ( ( HAL_GetTick() - start ) > timeout_ms )
            {
            return FLASH_TIMEOUT;
            }
        }
    }
}