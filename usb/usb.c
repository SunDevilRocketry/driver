/**
  ******************************************************************************
  * @file           : usb.c
  * @brief          : Universal Serial Driver Interface
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Hardware Assumptions:
  *        Target: STM32H733 (A0010, Rev 3 PCB)
  *        Legacy: STM32H750xx boards using UART-to-USB bridge (A0002, L0002, L0005)
  *
  *        For Native USB CDC (USE_USB_CDC_FS):
  *        - USB_OTG_HS enabled as Internal FS Phy (Device Only)
  *        - USB_DEVICE Middleware set to Communication Device Class (CDC)
  *        - USB clock must be exactly 48 MHz
  *        - VBUS sensing handled via USB_DETECT GPIO (configured as Input
  *          with Internal Pull-Down to prevent floating when unplugged)
  *
  *        For Legacy UART:
  *        - Standard UART TX/RX pins configured
  *        - Baud rate: 921600
  *
  *        Usage Example (Terminal Echo):
  *
  *        #include "usb.h"
  *
  *        int main(void) {
  *            #ifdef USE_USB_CDC_FS
  *                usb_init();
  *            #else
  *                usb_init(&huart3);
  *            #endif
  *
  *            uint8_t rx_byte;
  *            while (1) {
  *                if (usb_receive(&rx_byte, 1, 10) == USB_OK) {
  *                    usb_transmit(&rx_byte, 1, 10);
  *                }
  *            }
  *        }
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
 Standard Includes  
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
    #include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
    #include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER     )
    #include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION       )
    #include "sdr_pin_defines_A0005.h"
#elif defined( FLIGHT_COMPUTER_LITE )
    #include "sdr_pin_defines_A0007.h"
#elif defined( A0010                )
    #include "sdr_pin_defines_A0010.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"

#ifdef USE_USB_CDC_FS
    #include "usb_device.h"
    #include "usbd_cdc_if.h"
#endif /* USE_USB_CDC_FS */


/*------------------------------------------------------------------------------
 Preprocessor Directives 
------------------------------------------------------------------------------*/
#ifdef USE_USB_CDC_FS
    #define USB_FLUSH_RX_TIMEOUT_MS   10    /* Max wait to drain receive buffer  */
    #define USB_FLUSH_TX_TIMEOUT_MS   10    /* Max wait for TX completion        */
#endif /* USE_USB_CDC_FS */

/*------------------------------------------------------------------------------
 Global Variables                                                                  
------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/* Initializes USB middleware stack (CDC) or validates UART handle (legacy) */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_init(void) 
{
    /* API implementation */
    MX_USB_DEVICE_Init();
    return USB_OK;
}

#else
USB_STATUS usb_init(UART_HandleTypeDef* huart) 
{
    /* API implementation */
    if (huart == NULL) {
        return USB_FAIL;
    }
    return USB_OK;
}
#endif /* USE_USB_CDC_FS */


/* Transmits a specified number of bytes over USB */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout) 
{
    /* API implementation */
    if (tx_data_ptr == NULL || tx_data_size == 0 || !usb_detect()) {
        return USB_FAIL;
    }

    if (tx_data_size > APP_TX_DATA_SIZE) {
        return USB_FAIL;
    }

    uint32_t tickstart = HAL_GetTick();

    /* Wait for any ongoing transmission to complete */
    while (VCP_Is_Busy()) {
        if (!usb_detect() || (HAL_GetTick() - tickstart) > timeout) {
            VCP_Force_Unlock();
            return USB_TIMEOUT;
        }
    }

    /* Submit buffer to USB hardware */
    if (CDC_Transmit_HS(tx_data_ptr, tx_data_size) != USBD_OK) {
        return USB_FAIL;
    }

    /* Block until hardware finishes pushing the data */
    while (VCP_Is_Busy()) {
        if (!usb_detect() || (HAL_GetTick() - tickstart) > timeout) {
            VCP_Force_Unlock();
            return USB_TIMEOUT;
        }
    }

    return USB_OK;
}

#else
USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout) 
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Transmit( &( USB_HUART ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( usb_status != HAL_OK )
	{
	return usb_status;
	}
else
	{
	return USB_OK;
	}

} /* usb_transmit */
#endif /* USE_USB_CDC_FS */


/* Receives a specified number of bytes from the USB port */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_receive(void* rx_data_ptr, size_t rx_data_size, uint32_t timeout) 
{
    /* API implementation */
    if (!usb_detect() || rx_data_ptr == NULL || rx_data_size == 0) {
        return USB_FAIL;
    }

    uint32_t tickstart = HAL_GetTick();
    size_t   count     = 0;

    /* Poll buffer until requested byte count is met */
    while (count < rx_data_size) {
        if (!usb_detect()) {
            return USB_DISCONNECTED;
        }

        int16_t byte = VCP_Read_Byte();
        if (byte != -1) {
            ((uint8_t*)rx_data_ptr)[count++] = (uint8_t)byte;
        }

        if ((HAL_GetTick() - tickstart) > timeout) {
            return USB_TIMEOUT;
        }
    }

    return USB_OK;
}

#else
USB_STATUS usb_receive(void* rx_data_ptr, size_t rx_data_size, uint32_t timeout) 
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Receive bytes */
usb_status = HAL_UART_Receive( &( USB_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );

/* Return HAL status */
switch ( usb_status )
	{
	case HAL_TIMEOUT:
		{
		return USB_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return USB_OK;
		break;
		}
	default:
		{
		return USB_FAIL;
		break;
		}
	}

} /* usb_receive */
#endif /* USE_USB_CDC_FS */


/* Detects an active USB connection via VBUS GPIO */
#if defined( USE_USB_CDC_FS       ) || \
    defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
    defined( L0005_REV3           )
bool usb_detect(void) 
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
uint8_t usb_detect_pinstate;    /* USB detect state, return value from HAL    */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
usb_detect_pinstate = 0;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Read voltage on usb detect pin */
usb_detect_pinstate = HAL_GPIO_ReadPin( USB_DETECT_GPIO_PORT, USB_DETECT_PIN );

/* Set return value */
if ( usb_detect_pinstate == 0 )
	{
	return false;
	}
else
	{
	return true;
	}

} /* usb_detect */
#endif /* USE_USB_CDC_FS || A0002_REV2 || FLIGHT_COMPUTER_LITE || L0002_REV5 || L0005_REV3 */


/* Flushes pending data from the receive buffer, optionally waits for TX */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_flush(bool flush_tx) 
{
    /* API implementation */
    if (!usb_detect()) {
        return USB_FAIL;
    }

    /* Discard all pending bytes in receive buffer */
    uint32_t tickstart = HAL_GetTick();
    while (VCP_Bytes_Available() > 0) {
        VCP_Read_Byte();
        if ((HAL_GetTick() - tickstart) > USB_FLUSH_RX_TIMEOUT_MS) {
            break;
        }
    }

    /* If requested, wait for pending transmission to complete */
    if (flush_tx) {
        tickstart = HAL_GetTick();
        while (VCP_Is_Busy()) {
            if (!usb_detect() || (HAL_GetTick() - tickstart) > USB_FLUSH_TX_TIMEOUT_MS) {
                VCP_Force_Unlock();
                return USB_TIMEOUT;
            }
        }
    }

    return USB_OK;
}

#else
void usb_flush(void) 
{
/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Cycle receive register until no data remains */
while ( __HAL_UART_GET_FLAG( &( USB_HUART ), UART_FLAG_RXNE ) )
    {
    __HAL_UART_FLUSH_DRREGISTER( &( USB_HUART ) );
    }

} /* usb_flush */
#endif /* USE_USB_CDC_FS */
*******************************************************************************/
