/**
  ******************************************************************************
  * @file           : usb_uart.c
  * @brief          : Legacy UART-to-USB Serial Implementation
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  This file is conditionally included by usb.c when USE_USB_CDC_FS is
  *        NOT defined. It provides the UART-backed implementation of the USB
  *        driver API for legacy boards using a UART-to-USB bridge.
  *
  *        Do NOT compile this file directly. It is included as a translation
  *        unit by usb.c based on the active build configuration.
  *
  *        Legacy board support:
  *        - A0002 (Flight Computer)        → UART-to-USB bridge
  *        - L0002 (Engine Controller)      → UART-to-USB bridge
  *        - L0005 (Valve Controller)       → UART-to-USB bridge
  *        - A0005 (Ground Station)         → UART-to-USB bridge
  *        - A0007 (Flight Computer Lite)   → UART-to-USB bridge
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


/* Transmits a specified number of bytes over USB */
USB_STATUS usb_transmit
	(
	void*    tx_data_ptr  ,
	size_t   tx_data_size ,
	uint32_t timeout
	)
{   /* ==================== API implementation =================== */

    HAL_StatusTypeDef usb_status;

    /* Transmit bytes over UART */
    usb_status = HAL_UART_Transmit( &( USB_HUART ),
                                    tx_data_ptr   , 
                                    tx_data_size  , 
                                    timeout );

    /* Return HAL status */
    switch ( usb_status ) {
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

} /* usb_transmit */


/* Receives a specified number of bytes from the USB port */
USB_STATUS usb_receive
	(
	void*    rx_data_ptr  ,
	size_t   rx_data_size ,
	uint32_t timeout
	)
{   /* ==================== API implementation =================== */

    HAL_StatusTypeDef usb_status;

    /* Receive bytes over UART */
    usb_status = HAL_UART_Receive( &( USB_HUART ),
                                rx_data_ptr   , 
                                rx_data_size  , 
                                timeout );

    /* Return HAL status */
    switch ( usb_status ) {
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


/* Transmits a specified number of bytes over USB */
USB_STATUS usb_transmit_IT 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size  /* Size of transmit data */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Transmit_IT( &( USB_HUART ),
                                tx_data_ptr   , 
                                tx_data_size );

/* Return HAL status */
if ( usb_status != HAL_OK )
	{
	return usb_status;
	}
else
	{
	return USB_OK;
	}

} /* usb_transmit_IT */


/* Receive bytes without blocking. */
USB_STATUS usb_receive_IT 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size  /* Size of the data to be received */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Receive_IT( &( USB_HUART ),
                               rx_data_ptr   , 
                               rx_data_size );

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

} /* usb_receive_IT */


/* Detects an active USB connection via VBUS GPIO */
#if defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
    defined( L0005_REV3           )
bool usb_detect( void )
{   /* ==================== API implementation =================== */
    /* Read voltage on USB detect pin and compare against GPIO Enum */
    if (HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_SET) {
        return true;
    } else {
        return false;
    }
} /* usb_detect */
#endif /* A0002_REV2 || FLIGHT_COMPUTER_LITE || L0002_REV5 || L0005_REV3 */


/* Flushes pending data from the receive buffer */
void usb_flush( void )
{   /* ==================== API implementation =================== */

    /* Cycle receive register until no data remains */
    while ( __HAL_UART_GET_FLAG( &( USB_HUART ), UART_FLAG_RXNE ) ){
        __HAL_UART_FLUSH_DRREGISTER( &( USB_HUART ) );
    }

} /* usb_flush */


/*******************************************************************************
* END OF usb_uart.c                                                            *
*******************************************************************************/
