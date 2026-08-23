/**
  ******************************************************************************
  * @file           : lora.h
  * @brief          : RFM95/96/97/98 LoRa Radio Driver Interface (DMA)
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Hardware Assumptions:
  *        Target: STM32H7, multi-platform (FLIGHT_COMPUTER / GROUND_STATION / A0010)
  *        Radio:  RFM96 (Semtech SX1276 family, LoRa mode only)
  *        Bus:    SPI, handle/pin set selected per-platform below
  *
  *        DMA & D-Cache Coherency (STM32H7):
  *          HAL_SPI_Transmit_DMA()/HAL_SPI_Receive_DMA() handle non-blocking
  *          FIFO bursts. Post-transfer, lora_process_async_cb() must
  *          invalidate the D-Cache region covering the rx buffer prior to
  *          consumption (SCB_InvalidateDCache_by_Addr).
  *
  *          On transaction error, HAL_SPI_ErrorCallback() must route to
  *          lora_process_async_error_cb() to force NSS high and release the
  *          SPI bus.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LORA_H
#define LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include "stm32h7xx_hal.h"

#if   defined( FLIGHT_COMPUTER )
    #include "sdr_pin_defines_A0002.h"
    #define LORA_IDLE_MODE  LORA_STANDBY_MODE
#elif defined( GROUND_STATION )
    #include "sdr_pin_defines_A0005.h"
    #define LORA_IDLE_MODE  LORA_RX_CONTINUOUS_MODE
#elif defined( A0010 )
    #include "sdr_pin_defines_A0010.h"
    #define LORA_IDLE_MODE  LORA_RX_CONTINUOUS_MODE
#else
    #error "lora.h requires one of FLIGHT_COMPUTER, GROUND_STATION, or A0010 to be defined"
#endif

/*------------------------------------------------------------------------------
 Macros
------------------------------------------------------------------------------*/

#define LORA_TIMEOUT                2000

/** @brief Short SPI timeout for writes made from ISR context. Bounds worst-case 
 *         ISR stall. [LORA_TIMEOUT is seperate, task-based] 
 */
#define LORA_ISR_TIMEOUT            ( 5U )

/** @brief RegIrqFlags (0x12) bit 5: set when the last received packet
 *         failed its payload CRC check. Write-1-to-clear. */
#define LORA_IRQ_PAYLOAD_CRC_ERROR  ( 0x20U )

/** @brief RegIrqFlags (0x12) bit 3: set when a transmit completes.
 *         Write-1-to-clear. */
#define LORA_IRQ_TX_DONE            ( 0x08U )

/** @brief Expected RegVersion (0x42) read-back value. Datasheet pg. 105:
 *         reset value 0x11 for the RFM95/96/97/98 
 */
#define LORA_ID_VERSION_VAL         ( 0x11U )

/* US ISM band frequencies, used in the code to prevent violating US law */
#define ISM_MAX_FREQ                928000
#define ISM_MIN_FREQ                902000

/* --- DMA buffer sizing / alignment --- */

/**
 * @brief Max LoRa payload per transaction - full 256-byte FIFO, shared
 *        between TX and RX (datasheet section 4.1.2.3). RegPayloadLength
 *        is a 7-0 field with 0 not permitted, so effective max is 255, but
 *        256 is used here since it's already 32-byte aligned.
 */
#define LORA_DMA_MAX_PAYLOAD_BYTES  ( 256U )

/** @brief Padded DMA size ensuring alignment with 32-byte Cortex-M7 cache line.
 *         Already a multiple of 32, but macro kept for consistency w/ DMA
 *         sizing convention in case the max payload changes. */
#define LORA_DMA_BUF_BYTES_ALIGNED  ( ( LORA_DMA_MAX_PAYLOAD_BYTES + 31U ) & ~31U )


/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/

/**
 * @brief Radio register addresses.
 *        Datasheet: https://www.mouser.com/datasheet/2/975/1463993415RFM95_96_97_98W-1858106.pdf
 * @note  LoRa mode only - FSK-mode opcodes intentionally omitted.
 */
typedef enum _LORA_REGISTER_ADDR {
   LORA_REG_FIFO_RW                    = 0x00,
   LORA_REG_OPERATION_MODE             = 0x01,
   LORA_REG_FREQ_MSB                   = 0x06,
   LORA_REG_FREQ_MSD                   = 0x07,
   LORA_REG_FREQ_LSB                   = 0x08,
   LORA_REG_PA_CONFIG                  = 0x09,
   LORA_REG_PA_RAMP                    = 0x0A,
   LORA_REG_OVER_CURRENT_PROT_CTRL     = 0x0B,
   LORA_REG_LNA_SETTINGS               = 0x0C,
   LORA_REG_FIFO_SPI_POINTER           = 0x0D,
   LORA_REG_FIFO_TX_BASE_ADDR          = 0x0E,
   LORA_REG_FIFO_RX_BASE_ADDR          = 0x0F,
   LORA_REG_FIFO_RX_BASE_CUR_ADDR      = 0x10,
   LORA_REG_IRQ_FLAGS_MASK             = 0x11,
   LORA_REG_IRQ_FLAGS                  = 0x12,
   LORA_REG_FIFO_RX_NUM_BYTES          = 0x13,
   LORA_REG_RX_HEADER_CNT_MSB          = 0x14,
   LORA_REG_RX_HEADER_CNT_LSB          = 0x15,
   LORA_REG_RX_PACKET_CNT_MSB          = 0x16,
   LORA_REG_RX_PACKET_CNT_LSB          = 0x17,
   LORA_REG_MODEM_STATUS               = 0x18,
   LORA_REG_PACKET_SNR                 = 0x19,
   LORA_REG_PACKET_RSSI                = 0x1A,
   LORA_REG_CURRENT_RSSI               = 0x1B,
   LORA_REG_HOP_CHANNEL                = 0x1C,
   LORA_REG_MODEM_CONFIG_1             = 0x1D,
   LORA_REG_MODEM_CONFIG_2             = 0x1E,
   LORA_REG_SYMB_TIMEOUT_LSB           = 0x1F,
   LORA_REG_PREAMBLE_MSB               = 0x20,
   LORA_REG_PREAMBLE_LSB               = 0x21,
   LORA_REG_PAYLOAD_LENGTH             = 0x22,
   LORA_REG_MAX_PAYLOAD_LENGTH         = 0x23,
   LORA_REG_HOP_PERIOD                 = 0x24,
   LORA_REG_FIFO_RX_BYTE_ADDR          = 0x25,
   LORA_REG_MODEM_CONFIG_3             = 0x26,
   LORA_REG_DIO_MAPPING_MODE_1         = 0x40,
   LORA_REG_DIO_MAPPING_MODE_2         = 0x41,
   LORA_REG_ID_VERSION                 = 0x42,
   LORA_REG_TCXO_OR_XTAL               = 0x4B,
   LORA_REG_PA_SETTINGS                = 0x4D,
   LORA_REG_FORMER_TEMP                = 0x5B,
   LORA_REG_AGC_REFERENCE              = 0x61,
   LORA_REG_AGC_THRESHOLD_1            = 0x62,
   LORA_REG_AGC_THRESHOLD_2            = 0x63,
   LORA_REG_AGC_THRESHOLD_3            = 0x64
} LORA_REGISTER_ADDR;

/** @brief Chip operating modes. */
typedef enum _LORA_CHIPMODE {
   LORA_SLEEP_MODE          = 0x00,
   LORA_STANDBY_MODE        = 0x01,
   LORA_FREQ_SYNTH_TX_MODE  = 0x02,
   LORA_TRANSMIT_MODE       = 0x03,
   LORA_FREQ_SYNTH_RX_MODE  = 0x04,
   LORA_RX_CONTINUOUS_MODE  = 0x05,
   LORA_RX_SINGLE_MODE      = 0x06,
   LORA_RX_CAD              = 0x07
} LORA_CHIPMODE;

/** @brief Standard status return codes for all LoRa driver operations. */
typedef enum _LORA_STATUS {
   LORA_OK = 0,
   LORA_FAIL,
   LORA_TRANSMIT_FAIL,
   LORA_RECEIVE_FAIL,
   LORA_TIMEOUT_FAIL,
   LORA_BUFFER_UNDERSIZED,
   LORA_READY,
   LORA_WAITING,
   LORA_USING_DEFAULTS,
   LORA_BUSY,
   LORA_INVALID_CMD
} LORA_STATUS;

/** @brief Spreading factor. Datasheet page 107. */
typedef enum _LORA_SPREADING_FACTOR {
   LORA_SPREAD_6  = 6,
   LORA_SPREAD_7  = 7,
   LORA_SPREAD_8  = 8,
   LORA_SPREAD_9  = 9,
   LORA_SPREAD_10 = 10,
   LORA_SPREAD_11 = 11,
   LORA_SPREAD_12 = 12
} LORA_SPREADING_FACTOR;

/** @brief Signal bandwidth. Datasheet page 106. */
typedef enum _LORA_BANDWIDTH {
   LORA_BANDWIDTH_7_8_KHZ   = 0x00,
   LORA_BANDWIDTH_10_4_KHZ  = 0x01,
   LORA_BANDWIDTH_15_6_KHZ  = 0x02,
   LORA_BANDWIDTH_20_8_KHZ  = 0x03,
   LORA_BANDWIDTH_31_25_KHZ = 0x04,
   LORA_BANDWIDTH_41_7_KHZ  = 0x05,
   LORA_BANDWIDTH_62_5_KHZ  = 0x06,
   LORA_BANDWIDTH_125_KHZ   = 0x07,
   LORA_BANDWIDTH_250_KHZ   = 0x08,
   LORA_BANDWIDTH_500_KHZ   = 0x09
} LORA_BANDWIDTH;

/** @brief Error coding rate (denominator of 4/N). */
typedef enum _LORA_ERROR_CODING {
   LORA_ECR_4_5 = 0x01,
   LORA_ECR_4_6 = 0x02,
   LORA_ECR_4_7 = 0x03,
   LORA_ECR_4_8 = 0x04
} LORA_ERROR_CODING;

/** @brief Header mode. Datasheet page 106 (bit location); page 26-27 (meaning). */
typedef enum _LORA_HEADER_MODE {
   LORA_IMPLICIT_HEADER = 0b1,
   LORA_EXPLICIT_HEADER = 0b0
} LORA_HEADER_MODE;

/** @brief PA output pin selection. Datasheet pages 79 and 103. */
typedef enum _LORA_PA_SELECT {
   LORA_RFO      = 0x00,
   LORA_PA_BOOST = 0x01
} LORA_PA_SELECT;

/** @brief Full configuration struct, passed to lora_init(). */
typedef struct _LORA_CONFIG {
   LORA_CHIPMODE         lora_mode;         /* Current LoRa chip mode        */
   LORA_SPREADING_FACTOR lora_spread;       /* Spreading factor              */
   LORA_BANDWIDTH        lora_bandwidth;    /* Signal bandwidth              */
   LORA_ERROR_CODING     lora_ecr;          /* Error coding rate             */
   LORA_HEADER_MODE      lora_header_mode;  /* Header mode                  */
   LORA_PA_SELECT        lora_pa_select;    /* Amplifier selection           */
   uint32_t              lora_frequency;    /* Carrier frequency, in kHz.
                                                Chip units: (2^19 * x)/(32*10^3) */
} LORA_CONFIG;

/**
 * @brief Runtime preset subset of LORA_CONFIG (chipmode/header mode omitted -
 *        defined by firmware, not user-settable at runtime).
 */
typedef struct LORA_PRESET {
    uint8_t  lora_spread;      /* SF 6-12 supported. Validate range.            */
    uint8_t  lora_bandwidth;   /* enum -- LORA_BANDWIDTH, packed to one byte.   */
    uint8_t  lora_ecr;         /* 4:5, 4:6, 4:7, or 4:8                         */
    uint8_t  high_power_mode;  /* true: +20 dBm boost                          */
    uint32_t lora_frequency;   /* frequency in kHz                              */
} LORA_PRESET;
_Static_assert( sizeof( LORA_PRESET ) == 8, "LORA PRESET SIZE MISMATCH." );

/** @brief Uplink subcommand codes for preset upload/download. */
typedef enum LORA_SUBCMD_CODES {
    LORA_PRESET_UPLOAD   = 0x01,
    LORA_PRESET_DOWNLOAD = 0x02,
    LORA_PROBE_VERSION   = 0x03   /* diagnostic: read RegVersion over SPI */
} LORA_SUBCMD_CODES;

/* (add DMA-specific typedefs here as needed) */


/*------------------------------------------------------------------------------
 Function Prototypes
------------------------------------------------------------------------------*/

/* --- System Control & Blocking Init --- */

/**
 * @brief  Resets the LoRa modem via the hardware NSRESET pin.
 * @note   GPIO toggle only, no SPI transaction.
 */
void lora_reset
    (
    void
    );

/**
 * @brief  Sets the LoRa chip operating mode (sleep/standby/tx/rx/etc).
 * @note   Blocking read-modify-write of RegOpMode. Used during init and
 *         before/after async transactions (Commit 5.4).
 * @param  chip_mode: Target LORA_CHIPMODE.
 * @return LORA_STATUS
 */
LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    );

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
    );

/**
 * @brief  Applies a runtime LORA_PRESET (subset of full config).
 * @note   Blocking, infrequent - used for in-flight preset switching.
 * @param  preset: Pointer to preset struct.
 * @return LORA_STATUS
 */
LORA_STATUS lora_configure
    (
    LORA_PRESET* preset
    );

/**
 * @brief  Reads the modem's RegVersion (0x42) over SPI. Diagnostic only.
 * @note   Blocking, no reset, no mode change - a bare register read used to
 *         tell "SPI bus is dead / no module" apart from "config rejected".
 *         Expected value is LORA_ID_VERSION_VAL (0x11) for the SX1276.
 * @param  version: Destination for the raw register byte.
 * @return LORA_STATUS - LORA_FAIL if the SPI transaction itself failed.
 */
LORA_STATUS lora_probe_version
    (
    uint8_t* version
    );

/* --- Async TX --- */

/**
 * @brief  Launches a non-blocking transmit of buffer_ptr[0..buffer_len).
 * @note   Sequence: standby -> set FIFO TX pointer + payload length
 *         (polling) -> NSS low -> poll-write FIFO address byte -> DMA burst.
 *         NSS is asserted here but NOT released here - only
 *         lora_process_async_cb()/lora_process_async_error_cb() (running in
 *         ISR context) s.t. transaction isn't half torn down by call context.
 * @param  buffer_ptr: Payload to transmit.
 * @param  buffer_len: Payload length, 1-LORA_DMA_MAX_PAYLOAD_BYTES.
 * @return LORA_STATUS - LORA_BUSY if a transaction is already in flight,
 *         LORA_BUFFER_UNDERSIZED if buffer_len is 0 or oversized.
 */
LORA_STATUS lora_transmit_async
    (
    uint8_t* buffer_ptr,
    uint8_t  buffer_len
    );

/* --- ISR callbacks --- */

/**
 * @brief  DMA ISR Callback | call from HAL_SPI_TxCpltCallback() /
 *         HAL_SPI_TxRxCpltCallback().
 * @note   Runs in the DMA interrupt context. Handles hardware only (NSS,
 *         cache invalidation/clean, flags) to keep ISR latency minimal.
 */
void lora_process_async_cb
    (
    void
    );

/**
 * @brief  DMA Error ISR Callback | call from HAL_SPI_ErrorCallback().
 * @note   Recovers driver state after an SPI/DMA error during an async
 *         transfer started by lora_transmit_async() or
 *         lora_request_receive_async(). Forces NSS high so a failed LoRa
 *         DMA can't lock the shared SPI bus. Runs in interrupt context.
 *         Hardware-only (NSS, flags).
 */
void lora_process_async_error_cb
    (
    void
    );

/**
 * @brief  DIO0 EXTI ISR handler | call from HAL_GPIO_EXTI_Callback() on
 *         every DIO0 rising edge.
 * @note   DIO0 reports RxDone while in an RX mode but TxDone while in TX
 *         mode - same physical pin, meaning depends on current chip state.
 *         Internally dispatches to the TxDone path (return to standby) if
 *         a transmit is awaiting completion, otherwise treats the edge as
 *         RxDone and calls lora_request_receive_async(). Both paths are
 *         ISR-safe: no blocking register reads, only short-timeout blind
 *         writes built from lora_opmode_cache.
 */
void lora_process_dio0_cb
    (
    void
    );

/* --- Async RX --- */

/**
 * @brief  Launches a non-blocking receive of the last packet in the FIFO.
 * @note   Sequence: poll FifoRxCurrentAddr -> set FIFO pointer -> poll
 *         payload length -> check PayloadCrcError -> NSS low -> poll-write
 *         FIFO address byte -> full-duplex DMA burst (dummy TX, real RX).
 *         NSS ownership follows the same contract as lora_transmit_async():
 *         asserted here, only released by the ISR callbacks.
 * @return LORA_STATUS - LORA_BUSY if a transaction is already in flight.
 */
LORA_STATUS lora_request_receive_async
    (
    void
    );

/* --- RX consumer API --- */

/**
 * @brief  Returns true if a completed DMA receive is waiting to be consumed.
 * @retval true  - new data available (call lora_get_latest()).
 * @retval false - DMA in flight or no receive has completed yet.
 */
bool lora_has_new_data
    (
    void
    );

/**
 * @brief  Consumer - retrieves the latest completed receive.
 * @note   memcpy snapshot only - no parsing.
 * @param  buffer_ptr:         Destination buffer.
 * @param  buffer_len:         Destination buffer capacity.
 * @param  num_bytes_received: Out param, actual bytes copied (may be NULL).
 * @return LORA_STATUS - LORA_WAITING if no data is ready,
 *         LORA_BUFFER_UNDERSIZED if buffer_len is too small for the packet.
 */
LORA_STATUS lora_get_latest
    (
    uint8_t* buffer_ptr,
    uint8_t  buffer_len,
    uint8_t* num_bytes_received
    );


#ifdef __cplusplus
}
#endif
#endif /* LORA_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
