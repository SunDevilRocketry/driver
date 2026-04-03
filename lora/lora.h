/*******************************************************************************
*
* FILE:
* 		LoRa.h
*
* DESCRIPTION:
* 		Contains register definitions and functions for controlling the LoRa
*       radio module.
*
* COPYRIGHT:
*       Copyright (c) 2025 Sun Devil Rocketry.
*       All rights reserved.
*
*       This software is licensed under terms that can be found in the LICENSE
*       file in the root directory of this software component.
*       If no LICENSE file comes with this software, it is covered under the
*       BSD-3-Clause.
*
*       https://opensource.org/license/bsd-3-clause
*
*******************************************************************************/

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

#define LORA_TIMEOUT                2000

/* US ISM band frequencies, used in the code to prevent violating US law */
#define ISM_MAX_FREQ                928000
#define ISM_MIN_FREQ                902000

/*------------------------------------------------------------------------------
 Typdefs
------------------------------------------------------------------------------*/

/* Chip modes */
typedef enum _LORA_CHIPMODE {
   LORA_SLEEP_MODE = 0x00,
   LORA_STANDBY_MODE = 0x01,
   LORA_FREQ_SYNTH_TX_MODE = 0x02,
   LORA_TRANSMIT_MODE = 0x03,
   LORA_FREQ_SYNTH_RX_MODE = 0x04,
   LORA_RX_CONTINUOUS_MODE = 0x05,
   LORA_RX_SINGLE_MODE = 0x06,
   LORA_RX_CAD         = 0x07
} LORA_CHIPMODE;

/* Function return codes */
typedef enum _LORA_STATUS {
   LORA_OK = 0,
   LORA_FAIL,
   LORA_TRANSMIT_FAIL,
   LORA_RECEIVE_FAIL,
   LORA_TIMEOUT_FAIL,
   LORA_BUFFER_UNDERSIZED,
   LORA_READY,
   LORA_WAITING
} LORA_STATUS;

/* Datasheet page 107 */
typedef enum _LORA_SPREADING_FACTOR {
   LORA_SPREAD_6 = 6,
   LORA_SPREAD_7 = 7,
   LORA_SPREAD_8 = 8,
   LORA_SPREAD_9 = 9,
   LORA_SPREAD_10 = 10,
   LORA_SPREAD_11 = 11,
   LORA_SPREAD_12 = 12
} LORA_SPREADING_FACTOR;

/* Datasheet page 106 */
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

typedef enum _LORA_ERROR_CODING {
   LORA_ECR_4_5 = 0x01,
   LORA_ECR_4_6 = 0x02,
   LORA_ECR_4_7 = 0x03,
   LORA_ECR_4_8 = 0x04
} LORA_ERROR_CODING;

/* You can see this on 106 - actual meaings on pages 26 and 27 */
typedef enum _LORA_HEADER_MODE {
   LORA_IMPLICIT_HEADER = 0b1,
   LORA_EXPLICIT_HEADER = 0b0
} LORA_HEADER_MODE;


/* PaConfig options - See datasheet pages 79 and 103 */
typedef enum _LORA_PA_SELECT {
   LORA_RFO      = 0x00,
   LORA_PA_BOOST = 0x01
} LORA_PA_SELECT;

/* LORA CONFIG SETTINGS */
typedef struct _LORA_CONFIG {
   LORA_CHIPMODE lora_mode; // Current LORA Chipmode
   LORA_SPREADING_FACTOR lora_spread; // LoRa Spread factor
   LORA_BANDWIDTH lora_bandwidth; // Signal bandwith
   LORA_ERROR_CODING lora_ecr; // Data Error coding
   LORA_HEADER_MODE lora_header_mode; // LORA Header mode
   LORA_PA_SELECT lora_pa_select; // Amplifier Selection
   uint32_t lora_frequency; // The LORA carrier frequency, in kilohertz.
   // To convert to internal chip unit, use the formula (2^19 * x)/(32 * 10^3)
} LORA_CONFIG;

/*------------------------------------------------------------------------------
 Function Prototypes
------------------------------------------------------------------------------*/

LORA_STATUS lora_init
    (
    LORA_CONFIG *lora_config_ptr
    );

LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    );

// Reset the LoRa modem
void lora_reset
    (
    );

// Make LoRa radio transmitter
LORA_STATUS lora_transmit
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len
    );

// Check if modem has received packet
LORA_STATUS lora_receive_ready
    (
    );

// Read received packet if available
LORA_STATUS lora_receive
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len,
    uint8_t* num_bytes_received
    ) ;

#ifdef __cplusplus
}
#endif
#endif
