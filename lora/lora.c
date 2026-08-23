/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : RFM95/96/97/98 LoRa Radio Driver Implementation (DMA)
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note   See lora.h for architecture notes & hardware assumptions.
  *
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 Sun Devil Rocketry. All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause (https://opensource.org/license/bsd-3-clause).
  ******************************************************************************
  */

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdatomic.h>
#include "lora.h"



/*------------------------------------------------------------------------------
 Private Variables (double-buffers, atomics, indices)
------------------------------------------------------------------------------*/

/**
 * @brief SPI DMA buffers placed in D2 SRAM.
 *        - __ALIGNED(32): Prevents cache corruption during cache invalidation.
 *        - Section (.dma_buffer): Forces placement in RAM_D2 (0x30000000).
 *          DTCM (.bss) is not reachable by the DMA1/DMA2 controllers.
 *
 *        These hold payload bytes ONLY - the 1-byte SPI address/command is 
 *        sent via a separate polling transfer before the DMA burst is triggered 
 *        (NSS held low across both).
 */
__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t lora_dma_tx_buf[ LORA_DMA_BUF_BYTES_ALIGNED ];

__ALIGNED(32) __attribute__((section(".dma_buffer")))
static uint8_t lora_dma_rx_buf[ 2 ][ LORA_DMA_BUF_BYTES_ALIGNED ];

/**
 * @brief Synchronization flags.
 *        atomic_bool: both are written in interrupt context
 *        (lora_process_async_cb / lora_process_async_error_cb) and read
 *        from task/loop context.
 */
static atomic_bool lora_dma_busy  = false;
static atomic_bool lora_dma_ready = false;

/** @brief Number of valid payload bytes captured per rx buffer (indexed by
 *         buffer row). Populated by the RegFifoRxNumBytes poll. */
static uint8_t lora_dma_rx_bytes[ 2 ];

/**
 * @brief Index (0 or 1) of the lora_dma_rx_buf row targeted by an
 *        in-flight (or about-to-launch) DMA transfer.
 */
static uint8_t lora_dma_fill_idx = 0U;

/**
 * @brief Index of the lora_dma_rx_buf row holding the most recently
 *        completed, not-yet consumed transfer.
 */
static _Atomic uint8_t lora_dma_ready_idx = 0U;

/**
 * @brief Which operation the in-flight DMA transfer belongs to, so the
 *        single shared lora_process_async_cb() (called from both
 *        HAL_SPI_TxCpltCallback and HAL_SPI_TxRxCpltCallback) knows which
 *        completion path to run. Not atomic: only ever written by the task
 *        context b/f launching a DMA transfer (while lora_dma_busy is
 *        about to become true), and read once by the ISR that
 *        completes that same transfer.
 */
typedef enum _LORA_DMA_OP
    {
    LORA_DMA_OP_NONE = 0,
    LORA_DMA_OP_TX,
    LORA_DMA_OP_RX
    } LORA_DMA_OP;

static LORA_DMA_OP lora_dma_pending_op = LORA_DMA_OP_NONE;

/** 
 * @brief Shadow copy of RegOpMode s.t.ISR can bypass blocking SPI reads 
 *        during mode transitions.
 *        Synchronized w/ the hardware on every successful register write.
 */
static uint8_t lora_opmode_cache = 0U;

/**
 * @brief True between "TX FIFO DMA load complete, TX mode just keyed up"
 *        and "TxDone EXTI observed, back in standby". Lets
 *        lora_process_dio0_cb() tell a TxDone edge apart from an RxDone
 *        edge on the same physical DIO0 pin.
 */
static bool lora_dma_awaiting_txdone = false;


/*------------------------------------------------------------------------------
 Internal Function Prototypes
------------------------------------------------------------------------------*/

/* Blocking single-register access, each call owns both NSS edges */
static LORA_STATUS lora_reg_read
    (
    LORA_REGISTER_ADDR reg,
    uint8_t*            data
    );

static LORA_STATUS lora_reg_write
    (
    LORA_REGISTER_ADDR reg,
    uint8_t             data
    );

/* Reads RegVersion (0x42) - used by lora_init() to confirm the chip is
   present & responding before touching any other registers. */
static LORA_STATUS lora_get_device_id
    (
    uint8_t* device_id
    );

/* Address-byte-only polling helper (does NOT touch NSS)
   the caller holds NSS low across this call and the DMA burst that follows it. */
static LORA_STATUS lora_dma_send_addr_byte
    (
    LORA_REGISTER_ADDR reg,
    bool                write
    );

/* ISR-safe blind register write. Uses a micro-timeout to prevent bus 
   failures from stalling the CPU. Bypasses the read-back phase to 
   minimize interrupt latency. */
static LORA_STATUS lora_isr_blind_write
    (
    LORA_REGISTER_ADDR reg,
    uint8_t             data
    );

/* ISR-context register read. Same wire format as lora_reg_read() but
   LORA_ISR_TIMEOUT instead of LORA_TIMEOUT [used by lora_request_receive_async()] */
static LORA_STATUS lora_isr_reg_read
    (
    LORA_REGISTER_ADDR reg,
    uint8_t*            data
    );


/*------------------------------------------------------------------------------
 Internal Helpers
------------------------------------------------------------------------------*/

/**
 * @brief  Blocking read of a single internal modem register.
 * @note   Owns both NSS edges. Address byte MSB=0 selects a read
 *         (datasheet: "address bit is set to 0 for a Read access").
 * @param  reg:  Register address (7-bit).
 * @param  data: Destination for the read byte.
 * @return LORA_STATUS
 */
static LORA_STATUS lora_reg_read
    (
    LORA_REGISTER_ADDR reg,
    uint8_t*            data
    )
{
uint8_t addr_byte;

addr_byte = (uint8_t)( reg & 0x7FU );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( HAL_SPI_Transmit( &(LORA_SPI), &addr_byte, 1, LORA_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

if ( HAL_SPI_Receive( &(LORA_SPI), data, 1, LORA_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return LORA_OK;
} /* lora_reg_read */


/**
 * @brief  Blocking write of a single internal modem register.
 * @note   Owns both NSS edges. Address byte MSB=1 selects a write
 *         (datasheet: "address bit is set to 1 for a Write access").
 * @param  reg:  Register address (7-bit).
 * @param  data: Byte to write.
 * @return LORA_STATUS
 */
static LORA_STATUS lora_reg_write
    (
    LORA_REGISTER_ADDR reg,
    uint8_t             data
    )
{
uint8_t tx_buf[2];

tx_buf[0] = (uint8_t)( reg | 0x80U );
tx_buf[1] = data;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( HAL_SPI_Transmit( &(LORA_SPI), tx_buf, 2, LORA_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return LORA_OK;
} /* lora_reg_write */


/**
 * @brief  Reads RegVersion to confirm the chip is present and responding.
 * @param  device_id: Destination for the read byte (expect LORA_ID_VERSION_VAL).
 * @return LORA_STATUS
 */
static LORA_STATUS lora_get_device_id
    (
    uint8_t* device_id
    )
{
return lora_reg_read( LORA_REG_ID_VERSION, device_id );
} /* lora_get_device_id */


/**
 * @brief  Sends the 1-byte SPI address + R/W bit, polling. Stays polling
 *         because 1-byte DMA setup overhead exceeds the transfer itself.
 * @note   Does NOT touch NSS. Caller must pull NSS low before calling and
 *         leave it low afterward (precedes a DMA burst)
 * @param  reg:   Register address (7-bit). FIFO DMA bursts always target
 *                LORA_REG_FIFO_RW.
 * @param  write: true selects a write (MSB=1), false a read (MSB=0).
 * @return LORA_STATUS
 */
static LORA_STATUS lora_dma_send_addr_byte
    (
    LORA_REGISTER_ADDR reg,
    bool                write
    )
{
uint8_t addr_byte;

addr_byte = write ? (uint8_t)( reg | 0x80U ) : (uint8_t)( reg & 0x7FU );

if ( HAL_SPI_Transmit( &(LORA_SPI), &addr_byte, 1, LORA_TIMEOUT ) != HAL_OK )
    {
    return LORA_FAIL;
    }

return LORA_OK;
} /* lora_dma_send_addr_byte */


/**
 * @brief  ISR-context register write
 * @param  reg:  Register address (7-bit).
 * @param  data: Full byte to write.
 * @return LORA_STATUS
 */
static LORA_STATUS lora_isr_blind_write
    (
    LORA_REGISTER_ADDR reg,
    uint8_t             data
    )
{
uint8_t tx_buf[2];

tx_buf[0] = (uint8_t)( reg | 0x80U );
tx_buf[1] = data;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( HAL_SPI_Transmit( &(LORA_SPI), tx_buf, 2, LORA_ISR_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return LORA_OK;
} /* lora_isr_blind_write */


/**
 * @brief  ISR-context register read - see internal prototype note.
 * @param  reg:  Register address (7-bit).
 * @param  data: Destination for the read byte.
 * @return LORA_STATUS
 */
static LORA_STATUS lora_isr_reg_read
    (
    LORA_REGISTER_ADDR reg,
    uint8_t*            data
    )
{
uint8_t addr_byte;

addr_byte = (uint8_t)( reg & 0x7FU );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( HAL_SPI_Transmit( &(LORA_SPI), &addr_byte, 1, LORA_ISR_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

if ( HAL_SPI_Receive( &(LORA_SPI), data, 1, LORA_ISR_TIMEOUT ) != HAL_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    return LORA_FAIL;
    }

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return LORA_OK;
} /* lora_isr_reg_read */


/*------------------------------------------------------------------------------
Blocking init / reset / chip mode / configure
------------------------------------------------------------------------------*/

/**
 * @brief  Resets the LoRa modem via the hardware NSRESET pin.
 * @note   Hold low 10 ms then release. GPIO toggle only, no SPI.
 *         (Above the SX1276 datasheet's minimum reset-low and
 *         post-reset stabilization requirements). 
 */
void lora_reset
    (
    void
    )
{
HAL_GPIO_WritePin( LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_RESET );
HAL_Delay( 10 );
HAL_GPIO_WritePin( LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_SET );
HAL_Delay( 10 );
} /* lora_reset */

/**
 * @brief  Sets the LoRa chip operating mode (sleep/standby/tx/rx/etc).
 * @note   Blocking read-modify-write of RegOpMode bits[2:0] (Mode). Bits
 *         [7:3] (LongRangeMode, modulation type, reserved) are preserved.
 * @param  chip_mode: Target LORA_CHIPMODE.
 * @return LORA_STATUS
 */
LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    )
{
uint8_t opmode_reg;
uint8_t new_opmode_reg;

if ( lora_reg_read( LORA_REG_OPERATION_MODE, &opmode_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }

new_opmode_reg = (uint8_t)( ( opmode_reg & ~0x07U ) | (uint8_t)chip_mode );

if ( lora_reg_write( LORA_REG_OPERATION_MODE, new_opmode_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }

lora_opmode_cache = new_opmode_reg;

return LORA_OK;
} /* lora_set_chip_mode */

/**
 * @brief  Initializes the LoRa modem (Blocking).
 * @note   Applies frequency/spreading-factor/bandwidth/ECR/header/PA settings
 *         from lora_config_ptr. Enforces US ISM band limits.
 * @param  lora_config_ptr: Pointer to user configuration struct.
 * @return LORA_STATUS
 */
LORA_STATUS lora_init
    (
    LORA_CONFIG *lora_config_ptr
    )
{
uint8_t  device_id;
uint32_t bandwidth_hz;
uint8_t  opmode_reg;
uint8_t  config2_reg;
uint8_t  config1_reg;
uint32_t freq_mhz;
uint32_t freq_khz;
uint32_t frf_reg;
uint8_t  pa_config_reg;

if ( lora_get_device_id( &device_id ) != LORA_OK || device_id != LORA_ID_VERSION_VAL )
    {
    return LORA_FAIL;
    }

/* Map bandwidth enum to Hz for the ISM legality check below */
switch ( lora_config_ptr->lora_bandwidth )
    {
    case LORA_BANDWIDTH_7_8_KHZ:   bandwidth_hz = 7800;   break;
    case LORA_BANDWIDTH_10_4_KHZ:  bandwidth_hz = 10400;  break;
    case LORA_BANDWIDTH_15_6_KHZ:  bandwidth_hz = 15600;  break;
    case LORA_BANDWIDTH_20_8_KHZ:  bandwidth_hz = 20800;  break;
    case LORA_BANDWIDTH_31_25_KHZ: bandwidth_hz = 31250;  break;
    case LORA_BANDWIDTH_41_7_KHZ:  bandwidth_hz = 41700;  break;
    case LORA_BANDWIDTH_62_5_KHZ:  bandwidth_hz = 62500;  break;
    case LORA_BANDWIDTH_125_KHZ:   bandwidth_hz = 125000; break;
    case LORA_BANDWIDTH_250_KHZ:   bandwidth_hz = 250000; break;
    case LORA_BANDWIDTH_500_KHZ:   bandwidth_hz = 500000; break;
    default:
        return LORA_FAIL;
    }

/* Reject out-of-band configs before touching any register (US ISM 902-928 MHz) */
if ( ( lora_config_ptr->lora_frequency * 1000U + ( bandwidth_hz / 2U ) > ISM_MAX_FREQ * 1000U ) ||
     ( lora_config_ptr->lora_frequency * 1000U - ( bandwidth_hz / 2U ) < ISM_MIN_FREQ * 1000U ) )
    {
    return LORA_FAIL;
    }

/* Sleep mode required to toggle the LongRangeMode (LoRa) bit - datasheet pg. 102 */
if ( lora_set_chip_mode( LORA_SLEEP_MODE ) != LORA_OK )
    {
    return LORA_FAIL;
    }

if ( lora_reg_read( LORA_REG_OPERATION_MODE, &opmode_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }
if ( lora_reg_write( LORA_REG_OPERATION_MODE, (uint8_t)( opmode_reg | 0x80U ) ) != LORA_OK )
    {
    return LORA_FAIL;
    }
lora_opmode_cache = (uint8_t)( opmode_reg | 0x80U );

/* RegModemConfig2: spreading factor in bits[7:4] */
if ( lora_reg_read( LORA_REG_MODEM_CONFIG_2, &config2_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }
config2_reg = (uint8_t)( ( config2_reg & 0x0FU ) | ( (uint8_t)lora_config_ptr->lora_spread << 4 ) );
if ( lora_reg_write( LORA_REG_MODEM_CONFIG_2, config2_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }

/* RegModemConfig1: bandwidth[7:4] | ECR[3:1] | header mode[0] */
config1_reg = (uint8_t)( ( (uint8_t)lora_config_ptr->lora_bandwidth << 4 ) |
                         ( (uint8_t)lora_config_ptr->lora_ecr << 1 )       |
                         (uint8_t)lora_config_ptr->lora_header_mode );
if ( lora_reg_write( LORA_REG_MODEM_CONFIG_1, config1_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }

/* FRF = freq(kHz) * 2^19 / 32000 kHz, split into MHz/kHz components (avoid overflow) */
freq_mhz = lora_config_ptr->lora_frequency / 1000U;
freq_khz = lora_config_ptr->lora_frequency - ( freq_mhz * 1000U );
frf_reg  = ( freq_mhz * 524288U / 32U ) + ( freq_khz * 524288U / 32000U );

if ( lora_reg_write( LORA_REG_FREQ_MSB, (uint8_t)( ( frf_reg << 8 )  >> 24 ) ) != LORA_OK )
    {
    return LORA_FAIL;
    }
if ( lora_reg_write( LORA_REG_FREQ_MSD, (uint8_t)( ( frf_reg << 16 ) >> 24 ) ) != LORA_OK )
    {
    return LORA_FAIL;
    }
if ( lora_reg_write( LORA_REG_FREQ_LSB, (uint8_t)( ( frf_reg << 24 ) >> 24 ) ) != LORA_OK )
    {
    return LORA_FAIL;
    }

/* PA select bit + max drive on the remaining bits, per prior driver */
pa_config_reg = ( lora_config_ptr->lora_pa_select == LORA_PA_BOOST ) ? 0xFFU : 0x7FU;
if ( lora_reg_write( LORA_REG_PA_CONFIG, pa_config_reg ) != LORA_OK )
    {
    return LORA_FAIL;
    }

if ( lora_set_chip_mode( lora_config_ptr->lora_mode ) != LORA_OK )
    {
    return LORA_FAIL;
    }

return LORA_OK;
} /* lora_init */

/**
 * @brief  Applies a runtime LORA_PRESET (subset of full config), or falls
 *         back to defaults if preset is NULL or looks uninitialized.
 * @note   Blocking used for in-flight preset switching.
 *         Sequence:
 *           - header mode / chip mode are firmware-fixed, not preset fields
 *           - an all-0x00 or all-0xFF preset buffer is treated as "absent"
 *           - preset->lora_ecr is stored as the literal denominator (5-8)
 *             and mapped to LORA_ERROR_CODING (1-4) via -4
 *           - always reset()+init() afterward, whether preset or defaults
 * @param  preset: Pointer to preset struct, or NULL for defaults.
 * @return LORA_STATUS - LORA_USING_DEFAULTS if preset was NULL/invalid,
 *         LORA_OK if the given preset was applied, LORA_FAIL on init failure.
 */
LORA_STATUS lora_configure
    (
    LORA_PRESET* preset
    )
{
LORA_CONFIG lora_config;
LORA_STATUS lora_status;
uint8_t     cmp_buf[sizeof( LORA_PRESET )];

memset( &lora_config, 0, sizeof( lora_config ) );
lora_config.lora_header_mode = LORA_EXPLICIT_HEADER;
lora_config.lora_mode        = LORA_RX_CONTINUOUS_MODE;

if ( preset != NULL )
    {
    memset( cmp_buf, 0x00, sizeof( LORA_PRESET ) );
    if ( memcmp( cmp_buf, preset, sizeof( LORA_PRESET ) ) == 0 )
        {
        preset = NULL;
        }
    }

if ( preset != NULL )
    {
    memset( cmp_buf, 0xFF, sizeof( LORA_PRESET ) );
    if ( memcmp( cmp_buf, preset, sizeof( LORA_PRESET ) ) == 0 )
        {
        preset = NULL;
        }
    }

if ( preset == NULL )
    {
    lora_config.lora_bandwidth = LORA_BANDWIDTH_125_KHZ;
    lora_config.lora_ecr       = LORA_ECR_4_5;
    lora_config.lora_frequency = 915000;
    lora_config.lora_spread    = LORA_SPREAD_12;
    lora_config.lora_pa_select = LORA_RFO;

    lora_status = LORA_USING_DEFAULTS;
    }
else
    {
    lora_config.lora_bandwidth = (LORA_BANDWIDTH)preset->lora_bandwidth;
    lora_config.lora_ecr       = (LORA_ERROR_CODING)( preset->lora_ecr - 4U );
    lora_config.lora_frequency = preset->lora_frequency;
    lora_config.lora_spread    = (LORA_SPREADING_FACTOR)preset->lora_spread;
    lora_config.lora_pa_select = preset->high_power_mode ? LORA_PA_BOOST : LORA_RFO;

    lora_status = LORA_OK;
    }

/* Disable DIO0 EXTI before reset()/init() - lora_reset() toggles the
   physical reset pin, and lora_init() walks through several inter. register states b/f
   the chip is fully config'd. */
HAL_NVIC_DisableIRQ( LORA_IO0_EXTI_IRQn );

lora_reset();

if ( lora_init( &lora_config ) != LORA_OK )
    {
    HAL_NVIC_EnableIRQ( LORA_IO0_EXTI_IRQn );
    return LORA_FAIL;
    }

HAL_NVIC_EnableIRQ( LORA_IO0_EXTI_IRQn );

return lora_status;
} /* lora_configure */


/**
 * @brief  Diagnostic read of RegVersion. See header for rationale.
 */
LORA_STATUS lora_probe_version
    (
    uint8_t* version
    )
{
return lora_reg_read( LORA_REG_ID_VERSION, version );
} /* lora_probe_version */


/*------------------------------------------------------------------------------
Async TX
------------------------------------------------------------------------------*/

/**
 * @brief  Launches a non-blocking transmit of buffer_ptr[0..buffer_len).
 * @note   NSS ownership: this function asserts NSS low and hands the bus to
 *         DMA, but doesn't releases it. Only lora_process_async_cb() (success)
 *         or lora_process_async_error_cb() (failure), both running in ISR
 *         context, are allowed to pull NSS high [to prevent half torns].
 *
 * @param  buffer_ptr: Payload to transmit.
 * @param  buffer_len: Payload length, 1-LORA_DMA_MAX_PAYLOAD_BYTES.
 * @return LORA_STATUS
 */
LORA_STATUS lora_transmit_async
    (
    uint8_t* buffer_ptr,
    uint8_t  buffer_len
    )
{
uint8_t tx_base_addr;

if ( buffer_len == 0U || buffer_len > LORA_DMA_MAX_PAYLOAD_BYTES )
    {
    return LORA_BUFFER_UNDERSIZED;
    }

if ( atomic_exchange( &lora_dma_busy, true ) )
    {
    return LORA_BUSY;
    }

if ( lora_set_chip_mode( LORA_STANDBY_MODE ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

if ( lora_reg_read( LORA_REG_FIFO_TX_BASE_ADDR, &tx_base_addr ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }
if ( lora_reg_write( LORA_REG_FIFO_SPI_POINTER, tx_base_addr ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }
if ( lora_reg_write( LORA_REG_PAYLOAD_LENGTH, buffer_len ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

memcpy( lora_dma_tx_buf, buffer_ptr, buffer_len );

/* Clean (flush) the TX buffer so the DMA peripheral reads what was just
   written. */
#ifndef EMULATOR
SCB_CleanDCache_by_Addr( (uint32_t*)lora_dma_tx_buf,
                          (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
#endif

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( lora_dma_send_addr_byte( LORA_REG_FIFO_RW, true ) != LORA_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

lora_dma_pending_op = LORA_DMA_OP_TX;

if ( HAL_SPI_Transmit_DMA( &(LORA_SPI), lora_dma_tx_buf, buffer_len ) != HAL_OK )
    {
    lora_dma_pending_op = LORA_DMA_OP_NONE;
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

/* DMA now owns the bus. NSS release, TX-mode transition, and clearing
   lora_dma_busy happen in lora_process_async_cb() / _error_cb(). */
return LORA_OK;
} /* lora_transmit_async */


/*------------------------------------------------------------------------------
ISR Callbacks
------------------------------------------------------------------------------*/

/**
 * @brief  DMA ISR Callback | call from HAL_SPI_TxCpltCallback() (TX path)
 *         and HAL_SPI_TxRxCpltCallback() (RX path, full-duplex receive).
 * @note   Runs in the DMA interrupt context. Fulfills the NSS-release
 *         contract established in lora_transmit_async().
 *         This code is allowed to pull NSS high after a transaction starts.
 */
void lora_process_async_cb
    (
    void
    )
{
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if ( lora_dma_pending_op == LORA_DMA_OP_RX )
    {
    /* Invalidate before touching lora_dma_rx_buf */
    #ifndef EMULATOR
    SCB_InvalidateDCache_by_Addr( (uint32_t*)lora_dma_rx_buf[ lora_dma_fill_idx ],
                                   (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
    #endif

    /* fill_idx becomes the ready row. Consumer (lora_get_latest()) reads 
       ready_idx/lora_dma_ready */
    lora_dma_ready_idx = lora_dma_fill_idx;
    atomic_store( &lora_dma_ready, true );
    }
else if ( lora_dma_pending_op == LORA_DMA_OP_TX )
    {
    /* FIFO loaded; initiating RF transmission. Uses a blind write 
       from the shadow register to avoid CPU stalls. lora_dma_busy remains 
       true until the hardware TxDone interrupt occurs. */
    lora_opmode_cache = (uint8_t)( ( lora_opmode_cache & ~0x07U ) | (uint8_t)LORA_TRANSMIT_MODE );
    lora_isr_blind_write( LORA_REG_OPERATION_MODE, lora_opmode_cache );
    lora_dma_awaiting_txdone = true;
    lora_dma_pending_op = LORA_DMA_OP_NONE;
    return;
    }

lora_dma_pending_op = LORA_DMA_OP_NONE;
atomic_store( &lora_dma_busy, false );
} /* lora_process_async_cb */

/**
 * @brief  DIO0-as-TxDone handler. Static: only reachable through
 *         lora_process_dio0_cb()'s dispatch.
 * @note   ISR-context. Both writes are blind (short LORA_ISR_TIMEOUT, no
 *         read) - clearing TxDone doesn't need a read-modify-write since
 *         it's the only IRQ bit expected set at this point, unlike the RX
 *         path's IRQ_FLAGS clear which has to preserve/consider others.
 */
static void lora_process_txdone_cb
    (
    void
    )
{
lora_isr_blind_write( LORA_REG_IRQ_FLAGS, LORA_IRQ_TX_DONE );

lora_opmode_cache = (uint8_t)( ( lora_opmode_cache & ~0x07U ) | (uint8_t)LORA_RX_CONTINUOUS_MODE );
lora_isr_blind_write( LORA_REG_OPERATION_MODE, lora_opmode_cache );

lora_dma_awaiting_txdone = false;
atomic_store( &lora_dma_busy, false );
} /* lora_process_txdone_cb */

/**
 * @brief  DMA Error ISR Callback | call from HAL_SPI_ErrorCallback().
 * @note   Execution context: DMA/SPI interrupt (ISR). Hardware-only
 *
 *         Required to prevent NSS assert low forever (deadlocking).
 *
 *         No valid data is assumed to be present after an error, so
 *         lora_dma_ready is left/cleared false rather than set true.
 */
void lora_process_async_error_cb
    (
    void
    )
{
/* De-assert NSS immediately to free the shared SPI bus */
HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

lora_dma_pending_op = LORA_DMA_OP_NONE;
atomic_store( &lora_dma_busy, false );
atomic_store( &lora_dma_ready, false );
} /* lora_process_async_error_cb */

/**
 * @brief  DIO0 EXTI ISR handler | call from HAL_GPIO_EXTI_Callback() on
 *         every DIO0 rising edge.
 */
void lora_process_dio0_cb
    (
    void
    )
{
if ( lora_dma_awaiting_txdone )
    {
    lora_process_txdone_cb();
    }
else
    {
    lora_request_receive_async();
    }
} /* lora_process_dio0_cb */


/*------------------------------------------------------------------------------
Async RX / Consumer
------------------------------------------------------------------------------*/

/**
 * @brief  Returns true if a completed DMA receive is waiting to be consumed.
 * @retval true  - new data available (call lora_get_latest()).
 * @retval false - DMA in flight or no receive has completed yet.
 */
bool lora_has_new_data
    (
    void
    )
{
return lora_dma_ready;
} /* lora_has_new_data */

/**
 * @brief  Consumer - retrieves the latest completed receive.
 * @note   memcpy snapshot only - no parsing.
 * @param  buffer_ptr:         Destination buffer.
 * @param  buffer_len:         Destination buffer capacity.
 * @param  num_bytes_received: Out param, actual bytes copied (may be NULL).
 * @return LORA_STATUS
 */
LORA_STATUS lora_get_latest
    (
    uint8_t* buffer_ptr,
    uint8_t  buffer_len,
    uint8_t* num_bytes_received
    )
{
uint8_t idx;
uint8_t rx_bytes;

if ( !lora_dma_ready )
    {
    return LORA_WAITING;
    }

idx      = lora_dma_ready_idx;
rx_bytes = lora_dma_rx_bytes[ idx ];

if ( rx_bytes > buffer_len )
    {
    return LORA_BUFFER_UNDERSIZED;
    }

memcpy( buffer_ptr, lora_dma_rx_buf[ idx ], rx_bytes );
lora_dma_ready = false;

if ( num_bytes_received != NULL )
    {
    *num_bytes_received = rx_bytes;
    }

return LORA_OK;
} /* lora_get_latest */

/**
 * @brief  Launches a non-blocking receive of the last packet in the FIFO.
 * @note   FifoRxCurrentAddr->FifoAddrPtr, the length poll, the CRC check,
 *         and now the actual DMA hand-off.
 * @return LORA_STATUS
 */
LORA_STATUS lora_request_receive_async
    (
    void
    )
{
uint8_t rx_current_addr;
uint8_t rx_num_bytes;
uint8_t irq_flags;

if ( atomic_exchange( &lora_dma_busy, true ) )
    {
    return LORA_BUSY;
    }

/* RegFifoRxCurrentAddr (0x10): start address of the last packet received.
   Must be copied into RegFifoAddrPtr (0x0D) before reading the FIFO */
if ( lora_isr_reg_read( LORA_REG_FIFO_RX_BASE_CUR_ADDR, &rx_current_addr ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }
if ( lora_isr_blind_write( LORA_REG_FIFO_SPI_POINTER, rx_current_addr ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

/* RegRxNbBytes (0x13): number of payload bytes in the last packet - the
   length parameter the upcoming DMA burst needs. */
if ( lora_isr_reg_read( LORA_REG_FIFO_RX_NUM_BYTES, &rx_num_bytes ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }
if ( rx_num_bytes == 0U || rx_num_bytes > LORA_DMA_MAX_PAYLOAD_BYTES )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

/* RegIrqFlags (0x12): read once, write back unconditionally to clear
   whatever is currently set (RxDone, ValidHeader, PayloadCrcError, etc.) */
if ( lora_isr_reg_read( LORA_REG_IRQ_FLAGS, &irq_flags ) != LORA_OK )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }
lora_isr_blind_write( LORA_REG_IRQ_FLAGS, irq_flags );

if ( ( irq_flags & LORA_IRQ_PAYLOAD_CRC_ERROR ) != 0U )
    {
    atomic_store( &lora_dma_busy, false );
    return LORA_RECEIVE_FAIL;
    }

/* Target the buffer row NOT currently holding an unconsumed ready packet */
lora_dma_fill_idx ^= 1U;
lora_dma_rx_bytes[ lora_dma_fill_idx ] = rx_num_bytes;

/* Dummy TX bytes to clock the SPI during the full-duplex receive - the
   real address byte was already sent (polling) below. */
memset( lora_dma_tx_buf, 0x00U, rx_num_bytes );
#ifndef EMULATOR
SCB_CleanDCache_by_Addr( (uint32_t*)lora_dma_tx_buf,
                          (int32_t)LORA_DMA_BUF_BYTES_ALIGNED );
#endif

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

if ( lora_dma_send_addr_byte( LORA_REG_FIFO_RW, false ) != LORA_OK )
    {
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

lora_dma_pending_op = LORA_DMA_OP_RX;

if ( HAL_SPI_TransmitReceive_DMA( &(LORA_SPI),
                                   lora_dma_tx_buf,
                                   lora_dma_rx_buf[ lora_dma_fill_idx ],
                                   rx_num_bytes ) != HAL_OK )
    {
    lora_dma_pending_op = LORA_DMA_OP_NONE;
    HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );
    atomic_store( &lora_dma_busy, false );
    return LORA_FAIL;
    }

/* DMA now owns the bus. NSS release, cache invalidate, ready-buffer
   publication, and clearing lora_dma_busy happen in
   lora_process_async_cb() / _error_cb(). */
return LORA_OK;
} /* lora_request_receive_async */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
