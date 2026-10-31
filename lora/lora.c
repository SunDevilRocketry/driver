/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : Interfaces for wireless communication.
  ******************************************************************************
  * @copyright
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
                      ##### Integration Guide #####
  ==============================================================================
  [..]
  ## Dependencies:
  - None

  ## Invariants (contract):
  - The project must call lora_init() or lora_configure() before attempting
    to use this library.
  - The hardware must have an RFM95.
  - This file's abstractions must not be broken as all assumptions rely on
    the public interfaces being used for interaction.
  - To use the DMA API, the project must route these callbacks for LORA_SPI
    and the DIO0 EXTI line (the driver releases NSS itself):
    - HAL_SPI_TxCpltCallback, HAL_SPI_TxRxCpltCallback -> lora_process_async_cb
    - HAL_SPI_ErrorCallback -> lora_process_async_error_cb
    - HAL_GPIO_EXTI_Callback (LORA_IO0_PIN) -> lora_process_dio0_cb
  - DMA buffers live in the .dma_buffer linker section (D2 SRAM), since DTCM
    is not reachable by DMA1/DMA2.

  ## Instructions:
  - Call lora_init or lora_configure to set up the modem
  - Blocking: lora_transmit, lora_receive_ready, lora_receive
  - DMA: lora_transmit_async, lora_has_new_data, lora_get_latest
  ******************************************************************************
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdatomic.h>

#include "pindefs.h"

#include "lora.h"

#define LORA_BLOCKING_TX_TIMEOUT 50

/* Mode to return to after a DMA transmit completes */
#if defined( FLIGHT_COMPUTER )
    #define LORA_IDLE_MODE  LORA_STANDBY_MODE
#else
    #define LORA_IDLE_MODE  LORA_RX_CONTINUOUS_MODE
#endif

/* Globals -------------------------------------------------------------------*/
static LORA_STATUS lora_rx_done = LORA_WAITING;
static bool is_lora_configured = false;

/* DMA payload buffers; the address byte is sent by polling first, NSS held low */
__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t dma_tx_buf[ LORA_DMA_BUF_BYTES_ALIGNED ];

__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t dma_rx_buf[ 2 ][ LORA_DMA_BUF_BYTES_ALIGNED ];

typedef enum _LORA_DMA_OP {
    LORA_DMA_OP_NONE = 0,
    LORA_DMA_OP_TX,
    LORA_DMA_OP_RX
} LORA_DMA_OP;

/* Written in ISR context, read from the main loop */
static atomic_bool dma_busy  = false;
static atomic_bool dma_ready = false;
static _Atomic uint8_t dma_ready_idx = 0U;

static uint8_t     dma_rx_bytes[ 2 ];
static uint8_t     dma_fill_idx = 0U;
static LORA_DMA_OP dma_pending_op = LORA_DMA_OP_NONE;

/* Shadow of RegOpMode so ISRs can change mode without a blocking read */
static uint8_t opmode_cache = 0U;

/* DIO0 means TxDone while this is set, RxDone otherwise */
static bool dma_awaiting_txdone = false;

/* Static Prototypes ---------------------------------------------------------*/

static LORA_STATUS spi_receive
    (
    uint8_t* read_buffer_ptr
    );

static LORA_STATUS spi_transmit_byte
    (
    uint8_t byte
    );

static LORA_STATUS spi_transmit_data
    (
    LORA_REGISTER_ADDR reg,
    uint8_t data
    );

static LORA_STATUS read_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* pRegData
    );

static LORA_STATUS read_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* pRegData,
    uint8_t buffer_len
    );

static LORA_STATUS write_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t data
    );

static LORA_STATUS write_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data,
    uint8_t buffer_len
    );

static LORA_STATUS get_device_id
    (
    uint8_t* register_value
    );

static LORA_STATUS isr_read_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data
    );

static LORA_STATUS isr_write_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t data
    );

/* Procedures ----------------------------------------------------------------*/

/**
 * @brief Determine the initialization state of the LoRa modem.
 * 
 * @retval true LoRa is initialized.
 * @retval false LoRa is uninitialized.
 */
bool lora_is_lora_initialized
    (
    void
    )
{
return is_lora_configured;

} /* lora_is_lora_initialized */


/**
 * @brief Configure and initialize the LoRa modem.
 * 
 * @param preset A pointer to the LoRa preset struct.

 * @return LORA_STATUS The status of the LoRa modem.
 * @retval LORA_OK The modem initialized properly.
 * @retval LORA_USING_DEFAULTS The modem initialized
 * properly, but is using default values instead of the
 * preset parameter.
 */
LORA_STATUS lora_configure
    (
    LORA_PRESET* preset
    )
{
/* Local variables */
LORA_CONFIG lora_config;
LORA_STATUS lora_status = LORA_OK;
memset( &lora_config, 0, sizeof( lora_config ) );

/** Set app-dependent (non-configurable) parameters. 
  * Currently, the driver doesn't support changing
  * these values.
  */
lora_config.lora_header_mode = LORA_EXPLICIT_HEADER;
lora_config.lora_mode = LORA_STANDBY_MODE;

/* Make sure presets are neither all 0xFF nor all 0x00 for validity */
if( preset )
    {
    /* 00 buffer check */
    uint8_t cmp_buf[sizeof(LORA_PRESET)];
    memset(cmp_buf, 0x00, sizeof(LORA_PRESET));
    if (!memcmp(cmp_buf, preset, sizeof(LORA_PRESET)))
        {
        preset = NULL;
        }

    /* FF buffer check */
    memset(cmp_buf, 0xFF, sizeof(LORA_PRESET));
    if (preset && !memcmp(cmp_buf, preset, sizeof(LORA_PRESET)))
        {
        preset = NULL;
        }
    }

/* Check if presets exist. If so, we'll use them. Else, prepare defaults. */
if( !preset )
    {
    /* drew's most frequently tested parameters. in particular, 915 will help us see that these are the defaults */
    lora_config.lora_bandwidth = LORA_BANDWIDTH_125_KHZ;
    lora_config.lora_ecr = LORA_ECR_4_5;
    lora_config.lora_frequency = 915000;
    lora_config.lora_spread = LORA_SPREAD_12;
    lora_config.lora_pa_select = false;

    lora_status = LORA_USING_DEFAULTS;
    }
else
    {
    lora_config.lora_bandwidth = preset->lora_bandwidth;
    lora_config.lora_ecr = preset->lora_ecr - 4;
    lora_config.lora_frequency = preset->lora_frequency;
    lora_config.lora_spread = preset->lora_spread;
    lora_config.lora_pa_select = preset->high_power_mode;

    lora_status = LORA_OK;
    }

/* reset and re-initialize; DIO0 is masked so mid-init edges aren't read as RxDone/TxDone */
HAL_NVIC_DisableIRQ( LORA_IO0_EXTI_IRQn );
lora_reset();
HAL_Delay(10);
LORA_STATUS init_status = lora_init( &lora_config );
HAL_NVIC_EnableIRQ( LORA_IO0_EXTI_IRQn );

if (init_status == LORA_OK)
    {
    is_lora_configured = true;
    return lora_status;
    }
else
    {
    return LORA_FAIL;
    }

} /* lora_configure */


/**
 * @brief Set the operation mode for the modem.
 * 
 * @param chip_mode The operation mode to switch to.
 * @return LORA_STATUS The status of the modem.
 */
LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    )
{
/* Get initial value of the opmode register */
uint8_t operation_mode_register;
LORA_STATUS read_status = read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

if ( read_status != LORA_OK )
    {
    return LORA_FAIL;
    }

/* Change the register's operation mode */
uint8_t new_opmode_register = (operation_mode_register & ~(0x7));
new_opmode_register = (new_opmode_register | chip_mode);

/* Write back */
LORA_STATUS write_status = write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

if ( write_status + read_status == 0 )
    {
    opmode_cache = new_opmode_register;
    return LORA_OK;
    }
    
return LORA_FAIL;

} /* lora_set_chip_mode */


/**
 * @brief Initialize the lora modem
 * 
 * @param lora_config_ptr The configuration settings to initialize with
 * @return The status of the modem
 */
LORA_STATUS lora_init
    (
    LORA_CONFIG *lora_config_ptr
    )
{
/* Check device ID */
uint8_t device_id = 0;
get_device_id( &device_id );

if( device_id != LORA_ID_VERSION_VAL )
    {
    return LORA_FAIL;
    }

/** Check legality of frequency settings
  * We do this first so that nothing gets set if we're on an illegal frequency.
  */

/* Get a version of our bandwidth for legality calculations */
uint32_t bandwidth; /* Calculations down in hz due to decimal bandwidths */
switch( lora_config_ptr->lora_bandwidth ) {
    case LORA_BANDWIDTH_7_8_KHZ:
        bandwidth = 7800;
        break;
    case LORA_BANDWIDTH_10_4_KHZ:
        bandwidth = 10400;
        break;
    case LORA_BANDWIDTH_15_6_KHZ:
        bandwidth = 15600;
        break;
    case LORA_BANDWIDTH_20_8_KHZ:
        bandwidth = 20800;
        break;
    case LORA_BANDWIDTH_31_25_KHZ:
        bandwidth = 31250;
        break;
    case LORA_BANDWIDTH_41_7_KHZ:
        bandwidth = 41700;
        break;
    case LORA_BANDWIDTH_62_5_KHZ:
        bandwidth = 62500;
        break;
    case LORA_BANDWIDTH_125_KHZ:
        bandwidth = 125000;
        break;
    case LORA_BANDWIDTH_250_KHZ:
        bandwidth = 250000;
        break;
    case LORA_BANDWIDTH_500_KHZ:
        bandwidth = 500000;
        break;
    default:
        return LORA_FAIL;
}

/* Check legal compliance of frequency */
if ( !( lora_config_ptr->lora_frequency * 1000 + ( bandwidth / 2 ) <= ISM_MAX_FREQ * 1000
     && lora_config_ptr->lora_frequency * 1000 - ( bandwidth / 2 ) >= ISM_MIN_FREQ * 1000 ) ) 
    {
    return LORA_FAIL;
    }

LORA_STATUS set_sleep_status = lora_set_chip_mode( LORA_SLEEP_MODE ); // Switch to sleep mode to enable LoRa bit (datasheeet page 102)
// Get initial value of the operation mode register
uint8_t operation_mode_register;
LORA_STATUS read_status1 = read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

uint8_t new_opmode_register;
new_opmode_register = ( operation_mode_register | 0b10000000 ); // Toggle the LoRa bit

// Write new byte
LORA_STATUS write_status1 = write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

// Get initial value of config register 2
uint8_t modem_config2_register;
LORA_STATUS read_status2 = read_register( LORA_REG_RX_HEADER_INFO, &modem_config2_register );

uint8_t new_config2_register = modem_config2_register & 0x0F; // Erase spread factor bits
new_config2_register = ( new_config2_register | ( lora_config_ptr->lora_spread << 4 ) ); // Set the spread factor
LORA_STATUS write_status2 = write_register( LORA_REG_RX_HEADER_INFO, new_config2_register ); // Write new spread factor

// Get initial value of config register 1
uint8_t modem_config1_register;
LORA_STATUS read_status3 = read_register( LORA_REG_NUM_RX_BYTES, &modem_config1_register );
uint8_t new_config1_register = ( (lora_config_ptr->lora_bandwidth << 4) | (lora_config_ptr->lora_ecr << 1) | lora_config_ptr->lora_header_mode ); //TODO: Check datasheet for that last bit

// Write new config1 register
LORA_STATUS write_status3 = write_register( LORA_REG_NUM_RX_BYTES, new_config1_register );

// Determine register values for the frequency registers
uint32_t freq_mhz = lora_config_ptr->lora_frequency / 1000; // The megahertz component of our frequency.
uint32_t freq_khz = lora_config_ptr->lora_frequency - freq_mhz * 1000; // The kilohertz component of our frequency.
// The formula for converting khz to chip unit is fruqency * 524288 / ( 32 * 1000 )
// Straight up doing that causes an integer overflow, though, so I have to do it this way.
uint32_t frf_reg = ( freq_mhz * 524288 / 32 ) + ( freq_khz * 524288 / ( 32 * 1000 ) );

uint8_t lora_freq_reg1 = ( frf_reg <<  8 ) >> 24;
uint8_t lora_freq_reg2 = ( frf_reg << 16 ) >> 24;
uint8_t lora_freq_reg3 = ( frf_reg << 24 ) >> 24;

// Write the frequency registers
LORA_STATUS write_status4 = write_register( LORA_REG_FREQ_MSB, lora_freq_reg1 );
LORA_STATUS write_status5 = write_register( LORA_REG_FREQ_MSD, lora_freq_reg2 );
LORA_STATUS write_status6 = write_register( LORA_REG_FREQ_LSB, lora_freq_reg3 );

// Determine register values for the PA Config Register
uint8_t new_pa_select_reg;

if( lora_config_ptr->lora_pa_select )
    {
    new_pa_select_reg = 0b11111111; /* PA select, everything else maxed out */
    }
else
    {
    new_pa_select_reg = 0b01111111; /* RFO select, everything else maxed out */
    }

// Write the PA Config Register
LORA_STATUS write_status7 = write_register( LORA_REG_PA_CONFIG, new_pa_select_reg );

LORA_STATUS standby_status = lora_set_chip_mode( lora_config_ptr->lora_mode ); // Switch it into standby mode, which is what's convenient.

if( set_sleep_status + read_status1 + read_status2 + read_status3 + write_status1 + write_status2 + write_status3 + write_status4 + write_status5 + write_status6 + write_status7 + standby_status != 0 ) 
    {
    return LORA_FAIL;
    }
return LORA_OK;

} /* lora_init */


/**
 * @brief Reset LoRa modem
 */
void lora_reset
    (
    void
    )
{
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_RESET); // Pull Low
HAL_Delay(10);  // Hold reset low for 10 ms
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_SET);   // Pull High
HAL_Delay(10);  // Wait for SX1278 to stabilize

} /* lora_reset */


/**
 * @brief Transmit a packet (in blocking mode)
 * 
 * @param[in] buffer_ptr A pointer to the data to transmit
 * @param buffer_len The size of the buffer being transmitted
 * @return The status of the modem
 */
LORA_STATUS lora_transmit
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len
    )
{
/* Blocking TX/RX is planned to be removed in favor of the DMA API; until then, stay off the bus during DMA */
if ( atomic_load( &dma_busy ) )
    {
    return LORA_BUSY;
    }

// Mode request STAND-BY
LORA_STATUS standby_status = lora_set_chip_mode(LORA_STANDBY_MODE);

// Write data to LoRA FIFO
uint8_t fifo_ptr_addr;
LORA_STATUS tx_base_status = read_register(LORA_REG_FIFO_TX_BASE_ADDR, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
if (tx_base_status + standby_status != LORA_OK){
    // Error handler
    // led_set_color(// led_RED);
    return LORA_FAIL;
}
LORA_STATUS ptr_status = write_register(LORA_REG_FIFO_SPI_POINTER, fifo_ptr_addr); // Set fifo data pointer to TX base address
if (ptr_status != LORA_OK){
    // Error handler
    // led_set_color(// led_RED);
    return LORA_FAIL;
}

// Write buffer length to fifo_rw
LORA_STATUS fifo_status = write_register(LORA_REG_SIGNAL_TO_NOISE, buffer_len);

// Send byte to byte to the fifo buffer
LORA_STATUS sendbyte_status = LORA_OK;

sendbyte_status = write_register_buffer( LORA_REG_FIFO_RW, buffer_ptr, buffer_len );

LORA_STATUS tmode_status = lora_set_chip_mode(LORA_TRANSMIT_MODE);

uint8_t lora_op;
LORA_STATUS regop_status = LORA_OK;
uint32_t start_time = HAL_GetTick();
while ( ( start_time + ( LORA_BLOCKING_TX_TIMEOUT * buffer_len ) ) > HAL_GetTick() )
    {
    regop_status = read_register(LORA_REG_OPERATION_MODE, &lora_op);
    if ( ( ( lora_op & 0b111 ) == LORA_STANDBY_MODE )
      || ( regop_status != LORA_OK ) ) 
        {
        break;
        }
    }

if( fifo_status + tmode_status + regop_status + sendbyte_status == 0
 && ( start_time + ( LORA_BLOCKING_TX_TIMEOUT * buffer_len ) ) > HAL_GetTick() )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* lora_transmit */


/**
 * @brief Check if a packet has been received
 * 
 * @retval LORA_READY if a packet is ready
 * @retval LORA_WAITING if waiting for the packet
 * @retval LORA_FAIL if the receive failed
 */
LORA_STATUS lora_receive_ready
    (
    void
    )
{
/* Planned to be removed with blocking TX/RX; would clear the IRQ flags the DMA RX path relies on */
if ( atomic_load( &dma_busy ) )
    {
    return LORA_BUSY;
    }

uint8_t mode;

LORA_STATUS mode_check = read_register( LORA_REG_OPERATION_MODE, &mode );
mode = mode & 0x07;

if( mode_check == LORA_OK && mode == LORA_RX_CONTINUOUS_MODE ) {
    uint8_t irq_flag;

    LORA_STATUS irq_check = read_register(LORA_REG_IRQ_FLAGS, &irq_flag);

    if( irq_check == LORA_OK ) {
        write_register( LORA_REG_IRQ_FLAGS, irq_flag );
        uint8_t rx_done = ( irq_flag & 0x40 ) == 0x40;

        if( rx_done ) {
            lora_rx_done = LORA_READY;
            return LORA_READY;
        } else {
            lora_rx_done = LORA_WAITING;
            return LORA_WAITING;
        }
    } else {
        return LORA_FAIL;
    }
} else {
    return LORA_FAIL;
}
}


/**
 * @brief Receive a packet over LoRa
 * 
 * @param[in] buffer_ptr The buffer to receive into
 * @param buffer_len The length of the receive buffer
 * @param[out] num_bytes_received The actual number of bytes received
 * @return LORA_STATUS 
 */
LORA_STATUS lora_receive
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len,
    uint8_t* num_bytes_received
    )
{
/* Planned to be removed with blocking TX/RX */
if ( atomic_load( &dma_busy ) )
    {
    return LORA_BUSY;
    }

if ( lora_rx_done == LORA_READY ){
    // Write IRQ flags
    uint8_t irq_flag;

    LORA_STATUS irq_status2 = read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
    if( irq_status2 != LORA_OK )
        {
        return LORA_FAIL;
        }
    uint8_t crc_err = ( irq_flag & 0x20 ) == 0x20;

    if (!crc_err){ // TODO make a fail happen for a CRC error
        // Read received number of bytes
        uint8_t num_bytes;
        LORA_STATUS fifo2_status = read_register(LORA_REG_FIFO_RX_NUM_BYTES, &num_bytes);

        // In case SPI operation fials
        if( fifo2_status != LORA_OK ) {
            return LORA_FAIL;
        }

        if (num_bytes > buffer_len)
            {
            return LORA_BUFFER_UNDERSIZED;
            }

        // Set lora fifo pointer to the RX base current address
        uint8_t fifo_ptr_addr;
        LORA_STATUS base_adr_status = read_register(LORA_REG_FIFO_RX_BASE_CUR_ADDR, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
        if (base_adr_status != LORA_OK){
            // Error handler
            return LORA_FAIL;
        }
        LORA_STATUS ptr2_status = write_register(LORA_REG_FIFO_SPI_POINTER, fifo_ptr_addr); // Set fifo data pointer to TX base address
        if (ptr2_status != LORA_OK){
            // Error handler
            return LORA_FAIL;
        }
        // Begin extracting payload
        LORA_STATUS pld_xtr_status = LORA_OK;

        /*
        // Old Rx buffer read code
        // TODO don't remove until burst read is confirmed working
        for (int i = 0; i < num_bytes; i++){
            uint8_t packet;
            pld_xtr_status = read_register(LORA_REG_FIFO_RW, &packet);  // Access LoRA FIFO data buffer pointer
            buffer_ptr[i] = packet;
        } */

        read_register_buffer( LORA_REG_FIFO_RW, buffer_ptr, num_bytes);

        *num_bytes_received = num_bytes;
        if (pld_xtr_status == LORA_OK ) {
            return LORA_OK;
        } else {
            return LORA_FAIL;
        }
    }
    return LORA_OK;
} else if( lora_rx_done == LORA_WAITING ) {
    return LORA_WAITING;
}
return LORA_FAIL;
}

/**
 * @brief Read RegVersion over SPI. Diagnostic only: no reset, no mode change.
 *
 * @param[out] version The raw register byte (expect LORA_ID_VERSION_VAL).
 * @return LORA_FAIL if the SPI transaction itself failed.
 */
LORA_STATUS lora_probe_version
    (
    uint8_t* version
    )
{
return get_device_id( version );

} /* lora_probe_version */


/* DMA Procedures ------------------------------------------------------------*/

/**
 * @brief Start a DMA transmit of buffer_ptr[0..buffer_len).
 *
 * @note Asserts NSS and hands the bus to DMA. NSS is released only by
 * lora_process_async_cb() or lora_process_async_error_cb(). The busy flag
 * stays set until DIO0 reports TxDone.
 *
 * @param[in] buffer_ptr The payload to transmit.
 * @param buffer_len The payload length (1-255).
 * @retval LORA_BUSY A DMA transfer or transmission is still in flight.
 * @retval LORA_BUFFER_UNDERSIZED buffer_len is 0.
 */
LORA_STATUS lora_transmit_async
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len
    )
{
uint8_t tx_base_addr;

if ( buffer_len == 0U )
    {
    return LORA_BUFFER_UNDERSIZED;
    }

if ( atomic_exchange( &dma_busy, true ) )
    {
    return LORA_BUSY;
    }

/* FIFO can only be filled in standby (datasheet 4.1.6); DIO0 = TxDone needs Dio0Mapping 01 (Table 63) */
if ( lora_set_chip_mode( LORA_STANDBY_MODE ) != LORA_OK
  || read_register( LORA_REG_FIFO_TX_BASE_ADDR, &tx_base_addr ) != LORA_OK
  || write_register( LORA_REG_FIFO_SPI_POINTER, tx_base_addr ) != LORA_OK
  || write_register( LORA_REG_SIGNAL_TO_NOISE, buffer_len ) != LORA_OK /* 0x22 = RegPayloadLength */
  || write_register( LORA_REG_DIO_MAPPING_MODE_1, 0x40U ) != LORA_OK )
    {
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

memcpy( dma_tx_buf, buffer_ptr, buffer_len );
#if !defined( EMULATOR ) && !defined( UNIT_TEST )
SCB_CleanDCache_by_Addr( (uint32_t*)dma_tx_buf, (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
#endif

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( spi_transmit_byte( LORA_REG_FIFO_RW | 0x80 ) != LORA_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

dma_pending_op = LORA_DMA_OP_TX;

if ( HAL_SPI_Transmit_DMA( &(LORA_SPI), dma_tx_buf, buffer_len ) != HAL_OK )
    {
    dma_pending_op = LORA_DMA_OP_NONE;
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

return LORA_OK;

} /* lora_transmit_async */


/**
 * @brief Start a DMA read of the last packet received into the FIFO.
 *
 * @note Called from the DIO0 ISR on RxDone. Uses short-timeout register
 * access, then a full-duplex DMA burst into the free half of the double
 * buffer. NSS follows the same contract as lora_transmit_async().
 *
 * @retval LORA_BUSY A DMA transfer is still in flight.
 * @retval LORA_RECEIVE_FAIL The packet failed its payload CRC.
 */
LORA_STATUS lora_request_receive_async
    (
    void
    )
{
uint8_t rx_current_addr;
uint8_t rx_num_bytes;
uint8_t irq_flags;

if ( atomic_exchange( &dma_busy, true ) )
    {
    return LORA_BUSY;
    }

if ( isr_read_register( LORA_REG_FIFO_RX_BASE_CUR_ADDR, &rx_current_addr ) != LORA_OK
  || isr_write_register( LORA_REG_FIFO_SPI_POINTER, rx_current_addr ) != LORA_OK
  || isr_read_register( LORA_REG_FIFO_RX_NUM_BYTES, &rx_num_bytes ) != LORA_OK
  || rx_num_bytes == 0U
  || isr_read_register( LORA_REG_IRQ_FLAGS, &irq_flags ) != LORA_OK )
    {
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

/* Write back whatever is set to clear it (RxDone, ValidHeader, CRC error) */
isr_write_register( LORA_REG_IRQ_FLAGS, irq_flags );

if ( ( irq_flags & LORA_IRQ_PAYLOAD_CRC_ERROR ) != 0U )
    {
    atomic_store( &dma_busy, false );
    return LORA_RECEIVE_FAIL;
    }

/* Fill the row not holding an unconsumed packet */
dma_fill_idx ^= 1U;
dma_rx_bytes[ dma_fill_idx ] = rx_num_bytes;

/* Dummy bytes clock the SPI during the full-duplex read */
memset( dma_tx_buf, 0x00U, rx_num_bytes );
#if !defined( EMULATOR ) && !defined( UNIT_TEST )
SCB_CleanDCache_by_Addr( (uint32_t*)dma_tx_buf, (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
#endif

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( spi_transmit_byte( LORA_REG_FIFO_RW & 0x7F ) != LORA_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

dma_pending_op = LORA_DMA_OP_RX;

if ( HAL_SPI_TransmitReceive_DMA( &(LORA_SPI), dma_tx_buf, dma_rx_buf[ dma_fill_idx ], rx_num_bytes ) != HAL_OK )
    {
    dma_pending_op = LORA_DMA_OP_NONE;
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &dma_busy, false );
    return LORA_FAIL;
    }

return LORA_OK;

} /* lora_request_receive_async */


/**
 * @brief Check if a completed DMA receive is waiting.
 *
 * @retval true Data is ready for lora_get_latest().
 * @retval false A DMA is in flight or nothing has been received.
 */
bool lora_has_new_data
    (
    void
    )
{
return atomic_load( &dma_ready );

} /* lora_has_new_data */


/**
 * @brief Copy out the latest completed DMA receive.
 *
 * @param[out] buffer_ptr The destination buffer.
 * @param buffer_len The destination buffer capacity.
 * @param[out] num_bytes_received Bytes copied (may be NULL).
 * @retval LORA_WAITING No data is ready.
 * @retval LORA_BUFFER_UNDERSIZED buffer_len is smaller than the packet.
 */
LORA_STATUS lora_get_latest
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len,
    uint8_t* num_bytes_received
    )
{
if ( !atomic_load( &dma_ready ) )
    {
    return LORA_WAITING;
    }

uint8_t idx = dma_ready_idx;
uint8_t rx_bytes = dma_rx_bytes[ idx ];

if ( rx_bytes > buffer_len )
    {
    return LORA_BUFFER_UNDERSIZED;
    }

memcpy( buffer_ptr, dma_rx_buf[ idx ], rx_bytes );
atomic_store( &dma_ready, false );

if ( num_bytes_received != NULL )
    {
    *num_bytes_received = rx_bytes;
    }

return LORA_OK;

} /* lora_get_latest */


/**
 * @brief DMA complete ISR handler for LORA_SPI.
 *
 * @note TX: the FIFO is loaded, so key up transmit; busy stays set until
 * TxDone. RX: publish the filled buffer row.
 */
void lora_process_async_cb
    (
    void
    )
{
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if ( dma_pending_op == LORA_DMA_OP_TX )
    {
    opmode_cache = ( opmode_cache & ~0x07U ) | LORA_TRANSMIT_MODE;
    dma_awaiting_txdone = true;
    dma_pending_op = LORA_DMA_OP_NONE;
    isr_write_register( LORA_REG_OPERATION_MODE, opmode_cache );
    return;
    }

if ( dma_pending_op == LORA_DMA_OP_RX )
    {
    #if !defined( EMULATOR ) && !defined( UNIT_TEST )
    SCB_InvalidateDCache_by_Addr( (uint32_t*)dma_rx_buf[ dma_fill_idx ], (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
    #endif
    dma_ready_idx = dma_fill_idx;
    atomic_store( &dma_ready, true );
    }

dma_pending_op = LORA_DMA_OP_NONE;
atomic_store( &dma_busy, false );

} /* lora_process_async_cb */


/**
 * @brief DMA error ISR handler for LORA_SPI.
 *
 * @note Forces NSS high so a failed transfer can't hold the SPI bus.
 */
void lora_process_async_error_cb
    (
    void
    )
{
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

dma_pending_op = LORA_DMA_OP_NONE;
dma_awaiting_txdone = false;
atomic_store( &dma_busy, false );
atomic_store( &dma_ready, false );

} /* lora_process_async_error_cb */


/**
 * @brief DIO0 EXTI ISR handler.
 *
 * @note DIO0 is TxDone while a transmit is in flight (mapping 01), else RxDone (00).
 */
void lora_process_dio0_cb
    (
    void
    )
{
if ( !dma_awaiting_txdone )
    {
    lora_request_receive_async();
    return;
    }

isr_write_register( LORA_REG_IRQ_FLAGS, LORA_IRQ_TX_DONE );
isr_write_register( LORA_REG_DIO_MAPPING_MODE_1, 0x00U );

opmode_cache = ( opmode_cache & ~0x07U ) | LORA_IDLE_MODE;
isr_write_register( LORA_REG_OPERATION_MODE, opmode_cache );

dma_awaiting_txdone = false;
atomic_store( &dma_busy, false );

} /* lora_process_dio0_cb */

/* Internal Procedures -------------------------------------------------------*/

/**
 * @brief Receive a byte over SPI
 * 
 * @param[out] read_byte The byte to read
 * @return LORA_STATUS The status of the LoRa modem
 */
static LORA_STATUS spi_receive
    (
    uint8_t* read_byte
    )
{
/* Local Variables */
HAL_StatusTypeDef status;

/* Takes pointer to the read buffer. and puts output there */
status = HAL_SPI_Receive( &(LORA_SPI), read_byte, 1, LORA_TIMEOUT );

if ( status == HAL_OK )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* spi_receive */


/**
 * @brief Transmit a single byte over SPI.
 * 
 * @param byte The byte to transmit.
 * @return LORA_STATUS The status of the modem.
 */
static LORA_STATUS spi_transmit_byte
    (
    uint8_t byte
    )
{
/* Local Variables*/
HAL_StatusTypeDef status;

/* Takes register and data to write (1 byte) and writes that register. */
status = HAL_SPI_Transmit( &(LORA_SPI), &byte, 1, LORA_TIMEOUT);

if ( status == HAL_OK )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* spi_transmit_byte */


/**
 * @brief Write two bytes over SPI.
 * 
 * @param reg The register to write to (with 0x80 already OR'd in)
 * @param data The data to write.
 * @return LORA_STATUS The status of the modem.
 */
static LORA_STATUS spi_transmit_data
    (
    LORA_REGISTER_ADDR reg,
    uint8_t data
    )
{
/* Local Variables */
HAL_StatusTypeDef status;

/* Takes register and data to write and writes that register. */
uint8_t transmitBuffer[2] = { reg, data };
status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer[0], 2, LORA_TIMEOUT);

if ( status == HAL_OK )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* spi_transmit_data */


/**
 * @brief Read a LoRa register
 * 
 * @param lora_register The register to read from
 * @param[out] data The contents of that register
 * @return LORA_STATUS The status of the LoRa modem
 */
static LORA_STATUS read_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data
    )
{
/* Local variables */
LORA_STATUS transmit_status, receive_status;

/* Perform the read */
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

transmit_status = spi_transmit_byte( (lora_register & 0x7F) );
receive_status = spi_receive( data );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

/* Verify the read was performed successfully */
if ( ( transmit_status | receive_status ) == LORA_OK )
    {
    return LORA_OK;
    }
    
return LORA_FAIL;

} /* read_register */


/**
 * @brief Read a buffer from an internal LoRa register.
 * 
 * @param lora_register 
 * @param pRegData 
 * @param buffer_len 
 * @return LORA_STATUS 
 */
static LORA_STATUS read_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* pRegData,
    uint8_t buffer_len
    )
{
/* Local variables */
LORA_STATUS status;
HAL_StatusTypeDef hal_status;

/* Perform both steps in sequence so we deassert NSS no matter what */
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status = spi_transmit_byte( (lora_register & 0x7F) );
hal_status = HAL_SPI_Receive( &(LORA_SPI), pRegData, buffer_len, LORA_TIMEOUT );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if( hal_status != HAL_OK || status != LORA_OK )
    {
    return LORA_FAIL;
    }

return LORA_OK;

} /* read_register_buffer */


/**
 * @brief Write a LoRa modem register.
 * 
 * @param lora_register The register to write to
 * @param data The byte to write into the register
 * @return LORA_STATUS The status of the LoRa modem.
 */
static LORA_STATUS write_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t data
    )
{
LORA_STATUS status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status = spi_transmit_data( (lora_register | 0x80), data );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if ( status == LORA_OK )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* write_register */


/**
 * @brief Write a buffer to a register.
 * 
 * @param lora_register The register to write to.
 * @param data A buffer to write
 * @param buffer_len The length of the buffer
 * @return LORA_STATUS The status of the LoRa modem.
 */
static LORA_STATUS write_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data,
    uint8_t buffer_len
    )
{
/* Local Variables */
HAL_StatusTypeDef status = HAL_OK;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

/* Apply the write bitmask before transmitting */
uint8_t dest_reg = (lora_register | 0x80);
status |= HAL_SPI_Transmit( &(LORA_SPI), &dest_reg, 1, LORA_TIMEOUT);

/* Write buffer */
status |= HAL_SPI_Transmit( &(LORA_SPI), data, buffer_len, LORA_TIMEOUT);

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if ( status != HAL_OK )
    {
    return LORA_FAIL;
    }

return LORA_OK;

} /* write_register_buffer */

/**
 * @brief Helper to retrieve the device ID during initialization.
 * 
 * @param[out] register_value Device ID 
 * @return LORA_STATUS The status of the modem and this driver.
 */
static LORA_STATUS get_device_id
    (
    uint8_t* register_value
    )
{
LORA_STATUS status;

status = read_register( LORA_REG_ID_VERSION, register_value );

if ( status == LORA_OK )
    {
    return LORA_OK;
    }

return LORA_FAIL;

} /* get_device_id */


/**
 * @brief Read a register from ISR context (short timeout).
 *
 * @param lora_register The register to read from
 * @param[out] data The contents of that register
 * @return LORA_STATUS The status of the LoRa modem
 */
static LORA_STATUS isr_read_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data
    )
{
uint8_t addr = lora_register & 0x7F;
HAL_StatusTypeDef status = HAL_OK;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status |= HAL_SPI_Transmit( &(LORA_SPI), &addr, 1, LORA_ISR_TIMEOUT );
status |= HAL_SPI_Receive( &(LORA_SPI), data, 1, LORA_ISR_TIMEOUT );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return ( status == HAL_OK ) ? LORA_OK : LORA_FAIL;

} /* isr_read_register */


/**
 * @brief Write a register from ISR context (short timeout, no read-back).
 *
 * @param lora_register The register to write to
 * @param data The byte to write into the register
 * @return LORA_STATUS The status of the LoRa modem.
 */
static LORA_STATUS isr_write_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t data
    )
{
uint8_t buf[2] = { lora_register | 0x80, data };
HAL_StatusTypeDef status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status = HAL_SPI_Transmit( &(LORA_SPI), buf, 2, LORA_ISR_TIMEOUT );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return ( status == HAL_OK ) ? LORA_OK : LORA_FAIL;

} /* isr_write_register */

/**
 * End of file
 */