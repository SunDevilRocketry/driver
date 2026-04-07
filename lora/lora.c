/*******************************************************************************
*
* FILE:
* 		lora.c
*
* DESCRIPTION:
* 		Contains API functions for transmating and receiving from the board's
*       built-in LoRa module.
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


/*------------------------------------------------------------------------------
 Standard Includes
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 MCU Pins
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( GROUND_STATION    )
    #include "sdr_pin_defines_A0005.h"
#endif

/*------------------------------------------------------------------------------
 Project Inlcudes
------------------------------------------------------------------------------*/
#include "lora.h"
#include "main.h"
#include "usb.h"
#include "led.h"

/*------------------------------------------------------------------------------
 Global Variables
------------------------------------------------------------------------------*/
static LORA_STATUS lora_rx_done = LORA_WAITING;


/*------------------------------------------------------------------------------
    Internal function prototypes
------------------------------------------------------------------------------*/
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		LORA_SPI_Receive                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Wrapper function for HAL SPI transmit                                  *
*                                                                              *
*******************************************************************************/
static LORA_STATUS LORA_SPI_Receive
    (
    uint8_t* read_buffer_ptr
    )
{
/*------------------------------------------------------------------------------
    Local variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef status;

/*------------------------------------------------------------------------------
    Implementation
------------------------------------------------------------------------------*/

/* Takes pointer to the read buffer. and puts output there */
status = HAL_SPI_Receive( &(LORA_SPI), read_buffer_ptr, 1, LORA_TIMEOUT );

if ( status == HAL_OK )
    {
    /* Successful initializaiton */
    return LORA_OK;
    }

/* Failed initialization */
return LORA_FAIL;
}

static LORA_STATUS LORA_SPI_Transmit_Byte
    (
    uint8_t byte
    )
{
/*------------------------------------------------------------------------------
    Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef status;

/*------------------------------------------------------------------------------
    Implementation
------------------------------------------------------------------------------*/
/* Takes register and data to write (1 byte) and writes that register. */
status = HAL_SPI_Transmit( &(LORA_SPI), &byte, 1, LORA_TIMEOUT);

if (status == HAL_OK)
    {
    /* SPI transmpit successful */
    return LORA_OK;
    }

return LORA_FAIL;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		LORA_SPI_Transmit_Data                                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Transmit data over SPI                                                 *
*                                                                              *
*******************************************************************************/
static LORA_STATUS LORA_SPI_Transmit_Data
    (
    LORA_REGISTER_ADDR reg,
    uint8_t data
    )
{
HAL_StatusTypeDef status;

/* Takes register and data to write and writes that register. */
uint8_t transmitBuffer[2] = { reg, data };
status = HAL_SPI_Transmit( &(LORA_SPI), &transmitBuffer[0], 2, LORA_TIMEOUT);

if (status == HAL_OK){
    return LORA_OK;
} else return LORA_FAIL;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_read_register                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read internal modem register                                           *
*                                                                              *
*******************************************************************************/
static LORA_STATUS lora_read_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* pRegData
    )
{
LORA_STATUS transmit_status, receive_status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

transmit_status = LORA_SPI_Transmit_Byte( (lora_register & 0x7F) );
receive_status = LORA_SPI_Receive( pRegData );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if (transmit_status + receive_status == 0){
    return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_read_register_buffer                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read buffer from internal modem register                               *
*                                                                              *
*******************************************************************************/
static LORA_STATUS lora_read_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* pRegData,
    uint8_t buffer_len
    )
{
LORA_STATUS status;
HAL_StatusTypeDef hal_status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status = LORA_SPI_Transmit_Byte( (lora_register & 0x7F) );
if( status != LORA_OK )
    return LORA_FAIL;

hal_status = HAL_SPI_Receive( &(LORA_SPI), pRegData, buffer_len, LORA_TIMEOUT );
if( hal_status != HAL_OK )
    return LORA_FAIL;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

return LORA_OK;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_write_register                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Write internal modem register                                          *
*                                                                              *
*******************************************************************************/
static LORA_STATUS lora_write_register
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t data
    )
{
LORA_STATUS status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

status = LORA_SPI_Transmit_Data( (lora_register | 0x80), data );

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );

if ( status == LORA_OK )
    return LORA_OK;
else return LORA_FAIL;
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_write_register_buffer                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Write internal modem register with data buffer                         *
*                                                                              *
*******************************************************************************/
static LORA_STATUS lora_write_register_buffer
    (
    LORA_REGISTER_ADDR lora_register,
    uint8_t* data,
    uint8_t buffer_len
    )
{
HAL_StatusTypeDef status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );

// SPI write regester we are writing
uint8_t dest_reg = (lora_register | 0x80);
status = HAL_SPI_Transmit( &(LORA_SPI), &dest_reg, 1, LORA_TIMEOUT);
if ( status != HAL_OK )
    return LORA_FAIL;

// Write desire buffer
status = HAL_SPI_Transmit( &(LORA_SPI), data, buffer_len, LORA_TIMEOUT);
if ( status != HAL_OK )
    return LORA_FAIL;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );


return LORA_OK;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_cmd_execute                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Execute a LoRa terminal command.                                       *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_cmd_execute
    (
    uint8_t subcommand_code,
    LORA_PRESET* lora_preset_buf
    )
{
switch (subcommand_code)
    {
    /*-------------------------------------------------------------
     Upload Preset (to FC)
    -------------------------------------------------------------*/
    case LORA_PRESET_UPLOAD:
        {
        /* Recieve preset subcommand over USB */
        uint8_t data_receive_buffer[sizeof( LORA_PRESET )];
        if (usb_receive( data_receive_buffer,
                                sizeof( LORA_PRESET ),
                                10 * HAL_DEFAULT_TIMEOUT ) == USB_OK)
            {
            /* Copy received data into preset data */
            memcpy(lora_preset_buf, data_receive_buffer, sizeof( LORA_PRESET ) );
            return LORA_OK;
            }
        else
            {
            /* lora presets remain untouched if usb receive fails */
            return LORA_FAIL;
            }
        }
    /*-------------------------------------------------------------
     Download Preset (from FC)
    -------------------------------------------------------------*/
    case LORA_PRESET_DOWNLOAD:
        {
        /* tx straight from buffer (usb transmit does not modify the buffer) */
        if( usb_transmit( lora_preset_buf, sizeof( LORA_PRESET ), 10 * HAL_DEFAULT_TIMEOUT ) )
            {
            return LORA_OK;
            }
        else
            {
            return LORA_FAIL;
            }
        }
    /*-------------------------------------------------------------
     Unrecognized command code
    -------------------------------------------------------------*/
    default:
        {
        return LORA_FAIL;
        }
    }

} /* lora_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_configure                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Configure and re-initialize the lora modem.                            *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_configure
    (
    LORA_PRESET* preset
    )
{
LORA_CONFIG lora_config;
LORA_STATUS lora_status = LORA_OK;
memset( &lora_config, 0, sizeof( lora_config ) );

/* Set app-dependent (non-configurable) parameters. */
// ETS TEMP: We may elect to change these later, but this
// is what we're using for now.
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

/* reset and re-initialize */
lora_reset();
HAL_Delay(10);
if (lora_init( &lora_config ) == LORA_OK)
    {
    return lora_status;
    }
else
    {
    return LORA_FAIL;
    }

} /* lora_configure */

// Get the device chip ID
static LORA_STATUS lora_get_device_id
    (
    uint8_t* buffer_ptr
    )
{
LORA_STATUS status;

status = lora_read_register( LORA_REG_ID_VERSION, buffer_ptr );

if ( status == LORA_OK )
    return LORA_OK;
else return LORA_FAIL;
}


/*------------------------------------------------------------------------------
    Procedures
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_read_register_IT                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read internal modem register.                                          *
*                                                                              *
* NOTE:                                                                        *
*       A completion callback (HAL_SPI_TxRxCpltCallback) for this operation    *
*       MUST be registered for the LoRa SPI handle. The completion callback    *
*       must pull NSS high like so:                                            *
*                                                                              *
*       HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );   *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_read_register_IT
    (
    uint8_t lora_register,
    uint8_t* pRegData /* o: 2 byte array; first byte will be empty, second will have the data */
    )
{
HAL_StatusTypeDef hal_status;
static uint8_t read_reg[2];
read_reg[0] = lora_register & 0x7F;
read_reg[1] = 0x00;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );
hal_status = HAL_SPI_TransmitReceive_IT( &(LORA_SPI), read_reg, pRegData, 2 );

/* NSS high is in the callback */

if ( hal_status == HAL_OK ) 
    {
    return LORA_OK;
    }
else 
    {
    return LORA_FAIL;
    }

} /* lora_read_register_IT */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_write_register_IT                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Write to modem (register write).                                       *
*                                                                              *
* NOTE:                                                                        *
*       A completion callback (HAL_SPI_TxCpltCallback) for this operation      *
*       MUST be registered for the LoRa SPI handle. The completion callback    *
*       must pull NSS high like so:                                            *
*                                                                              *
*       HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );   *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_write_register_IT
    (
    uint8_t lora_register,
    uint8_t data
    )
{
HAL_StatusTypeDef hal_status;
static uint8_t write_reg[2]; /* statically scoped so it doesn't go out of scope during tx */
write_reg[0] = (lora_register | 0x80);
write_reg[1] = data;

return lora_write_IT(write_reg, 2);

} /* lora_write_register_IT */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_write_IT                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Write to modem (burst write, also used for register write).            *
*                                                                              *
* NOTE:                                                                        *
*       A completion callback (HAL_SPI_TxCpltCallback) for this operation      *
*       MUST be registered for the LoRa SPI handle. The completion callback    *
*       must pull NSS high like so:                                            *
*                                                                              *
*       HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_SET );   *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_write_IT
    (
    uint8_t* data,
    size_t   len
    )
{
HAL_StatusTypeDef hal_status;

HAL_GPIO_WritePin( LORA_NSS_GPIO_PORT, LORA_NSS_PIN, GPIO_PIN_RESET );
hal_status = HAL_SPI_Transmit_IT( &(LORA_SPI), data, len );

/* NSS high is in the callback */

if ( hal_status == HAL_OK ) 
    {
    return LORA_OK;
    }
else 
    {
    return LORA_FAIL;
    }

} /* lora_write_IT */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_set_chip_mode                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set operation mode of LoRa modem                                       *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_set_chip_mode
    (
    LORA_CHIPMODE chip_mode
    )
{
// Get initial value of the operation mode register
uint8_t operation_mode_register;
LORA_STATUS read_status = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

if (read_status != LORA_OK)
{
    return LORA_FAIL;
}

// // Fail if not in LORA Mode
// if ( !( operation_mode_register & (1<<7) ) ){
//     return LORA_FAIL;
// }

// Change the value of the chip register to set it to the suggested chip mode
uint8_t new_opmode_register = (operation_mode_register & ~(0x7));
new_opmode_register = (new_opmode_register | chip_mode);

// Write new byte
LORA_STATUS write_status = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

if ( write_status + read_status == 0 ){
    return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_init                                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize LoRa modem                                                  *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_init
    (
    LORA_CONFIG *lora_config_ptr
    )
{
// TODO add check for accurate chip ID (more than zero)
uint8_t device_id = 0;
lora_get_device_id( &device_id );

if( device_id == 0 ) {
    return LORA_FAIL;
}

// Check legality of frequency settings
// We do this first so that nothing gets set if we're on an illegal frequency.

// Get a version of our bandwidth for legality calculations
uint32_t bandwidth; // Calculations down in hz due to decimal bandwidths
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
        // Just in case, even though this is reading an enum
        return LORA_FAIL;
}

// Check legal compliance of frequency:
if( !( lora_config_ptr->lora_frequency * 1000 + ( bandwidth / 2 ) <= ISM_MAX_FREQ * 1000 &&
    lora_config_ptr->lora_frequency * 1000 - ( bandwidth / 2 ) >= ISM_MIN_FREQ * 1000 )
) {
    return LORA_FAIL;
}

LORA_STATUS set_sleep_status = lora_set_chip_mode( LORA_SLEEP_MODE ); // Switch to sleep mode to enable LoRa bit (datasheeet page 102)
// Get initial value of the operation mode register
uint8_t operation_mode_register;
LORA_STATUS read_status1 = lora_read_register( LORA_REG_OPERATION_MODE, &operation_mode_register );

uint8_t new_opmode_register;
new_opmode_register = ( operation_mode_register | 0b10000000 ); // Toggle the LoRa bit

// Write new byte
LORA_STATUS write_status1 = lora_write_register( LORA_REG_OPERATION_MODE, new_opmode_register );

// Get initial value of config register 2
uint8_t modem_config2_register;
LORA_STATUS read_status2 = lora_read_register( LORA_REG_RX_HEADER_INFO, &modem_config2_register );

uint8_t new_config2_register = modem_config2_register & 0x0F; // Erase spread factor bits
new_config2_register = ( new_config2_register | ( lora_config_ptr->lora_spread << 4 ) ); // Set the spread factor
LORA_STATUS write_status2 = lora_write_register( LORA_REG_RX_HEADER_INFO, new_config2_register ); // Write new spread factor

// Get initial value of config register 1
uint8_t modem_config1_register;
LORA_STATUS read_status3 = lora_read_register( LORA_REG_NUM_RX_BYTES, &modem_config1_register );
uint8_t new_config1_register = ( (lora_config_ptr->lora_bandwidth << 4) | (lora_config_ptr->lora_ecr << 1) | lora_config_ptr->lora_header_mode ); //TODO: Check datasheet for that last bit

// Write new config1 register
LORA_STATUS write_status3 = lora_write_register( LORA_REG_NUM_RX_BYTES, new_config1_register );

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
LORA_STATUS write_status4 = lora_write_register( LORA_REG_FREQ_MSB, lora_freq_reg1 );
LORA_STATUS write_status5 = lora_write_register( LORA_REG_FREQ_MSD, lora_freq_reg2 );
LORA_STATUS write_status6 = lora_write_register( LORA_REG_FREQ_LSB, lora_freq_reg3 );

// Determine register values for the PA Config Register
uint8_t pa_select_reg;
LORA_STATUS read_status4 = lora_read_register( LORA_REG_PA_CONFIG, &pa_select_reg );
uint8_t new_pa_select_reg =  pa_select_reg | lora_config_ptr->lora_pa_select;

// Write the PA Config Register
LORA_STATUS write_status7 = lora_write_register( LORA_REG_PA_CONFIG, new_pa_select_reg );

LORA_STATUS standby_status = lora_set_chip_mode( lora_config_ptr->lora_mode ); // Switch it into standby mode, which is what's convenient.

if( set_sleep_status + read_status1 + read_status2 + read_status3 + read_status4 + write_status1 + write_status2 + write_status3 + write_status4 + write_status5 + write_status6 + write_status7 + standby_status == 0 ) {
    return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_reset                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Reset LoRa modem (Initialization function needs to be called again)    *
*                                                                              *
*******************************************************************************/
void lora_reset
    (
    void
    )
{
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_RESET); // Pull Low
HAL_Delay(10);  // Hold reset low for 10 ms
HAL_GPIO_WritePin(LORA_RST_GPIO_PORT, LORA_RST_PIN, GPIO_PIN_SET);   // Pull High
HAL_Delay(10);  // Wait for SX1278 to stabilize
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_transmit                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       transmit a buffer through lora fifo                                    *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_transmit
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len
    )
{
uint8_t fifo_ptr_addr_test; // Testing fifo increment

// Mode request STAND-BY
LORA_STATUS standby_status = lora_set_chip_mode(LORA_STANDBY_MODE);

// Write data to LoRA FIFO
uint8_t fifo_ptr_addr;
LORA_STATUS tx_base_status = lora_read_register(LORA_REG_FIFO_TX_BASE_ADDR, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
if (tx_base_status + standby_status != LORA_OK){
    // Error handler
    // led_set_color(// led_RED);
    return LORA_FAIL;
}
LORA_STATUS ptr_status = lora_write_register(LORA_REG_FIFO_SPI_POINTER, fifo_ptr_addr); // Set fifo data pointer to TX base address
if (ptr_status != LORA_OK){
    // Error handler
    // led_set_color(// led_RED);
    return LORA_FAIL;
}

// Write buffer length to fifo_rw
LORA_STATUS fifo_status = lora_write_register(LORA_REG_SIGNAL_TO_NOISE, buffer_len);

// Send byte to byte to the fifo buffer
LORA_STATUS sendbyte_status = LORA_OK;

/*
// Old transmit buffer write code
// TODO don't remove until burst transmit is working
for (int i = 0; i<buffer_len; i++){
    sendbyte_status = lora_write_register(LORA_REG_FIFO_RW, buffer_ptr[i]);
}
*/

sendbyte_status = lora_write_register_buffer( LORA_REG_FIFO_RW, buffer_ptr, buffer_len );

LORA_STATUS tmode_status = lora_set_chip_mode(LORA_TRANSMIT_MODE);

uint8_t lora_op;
LORA_STATUS regop_status;
while (1){ // TODO Add a timeout here
    regop_status = lora_read_register(LORA_REG_OPERATION_MODE, &lora_op);
    if ((lora_op & 0b111) == LORA_STANDBY_MODE){
        break;
    }
}
if( fifo_status + tmode_status + regop_status + sendbyte_status == 0 ) {
        return LORA_OK;
} else {
    return LORA_FAIL;
}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_receive_ready                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Check if new packt has been received                                   *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_receive_ready
    (
    void
    )
{
uint8_t mode;

LORA_STATUS mode_check = lora_read_register( LORA_REG_OPERATION_MODE, &mode );
mode = mode & 0x07;

if( mode_check == LORA_OK && mode == LORA_RX_CONTINUOUS_MODE ) {
    uint8_t irq_flag;

    LORA_STATUS irq_check = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);

    if( irq_check == LORA_OK ) {
        lora_write_register( LORA_REG_IRQ_FLAGS, irq_flag );
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

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		lora_receive                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       lora_receive: receive a buffer from lora fifo with continuous mode     *
*                                                                              *
*******************************************************************************/
LORA_STATUS lora_receive
    (
    uint8_t* buffer_ptr,
    uint8_t buffer_len,
    uint8_t* num_bytes_received
    )
{
uint8_t timeout_flag;

if ( lora_rx_done == LORA_READY ){
    // Write IRQ flags
    uint8_t irq_flag;

    LORA_STATUS irq_status2 = lora_read_register(LORA_REG_IRQ_FLAGS, &irq_flag);
    uint8_t crc_err = ( irq_flag & 0x20 ) == 0x20;

    if (!crc_err){ // TODO make a fail happen for a CRC error
        // Read received number of bytes
        uint8_t num_bytes;
        LORA_STATUS fifo2_status = lora_read_register(LORA_REG_FIFO_RX_NUM_BYTES, &num_bytes);

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
        LORA_STATUS base_adr_status = lora_read_register(LORA_REG_FIFO_RX_BASE_CUR_ADDR, &fifo_ptr_addr);  // Access LoRA FIFO data buffer pointer
        if (base_adr_status != LORA_OK){
            // Error handler
            return LORA_FAIL;
        }
        LORA_STATUS ptr2_status = lora_write_register(LORA_REG_FIFO_SPI_POINTER, fifo_ptr_addr); // Set fifo data pointer to TX base address
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
            pld_xtr_status = lora_read_register(LORA_REG_FIFO_RW, &packet);  // Access LoRA FIFO data buffer pointer
            buffer_ptr[i] = packet;
        } */

        lora_read_register_buffer( LORA_REG_FIFO_SPI_POINTER, buffer_ptr, num_bytes);

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

/*------------------------------------------------------------------------------
    Internal procedures
------------------------------------------------------------------------------*/
