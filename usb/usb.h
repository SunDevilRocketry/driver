/**
  ******************************************************************************
  * @file           : usb.h
  * @brief          : Universal Serial Communication Driver
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  Provides a unified interface for both Native USB CDC (A0010) and
  *        Legacy UART-to-USB bridges via the USE_USB_CDC_FS compiler flag.
  *
  *        Features:
  *        - Consistent API regardless of underlying hardware
  *        - Blocking transmit/receive for synchronization reliability
  *        - Internal 2048-byte circular buffer for reception (CDC mode) [Check usbd_cdc_if.c]
  *        - Deadlock prevention via VCP_Force_Unlock on disconnect [Check usbd_cdc_if.c]
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
#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>

#ifndef USE_USB_CDC_FS
    #include "stm32h7xx_hal.h"  /* Required for UART_HandleTypeDef in legacy mode */
#endif


/*------------------------------------------------------------------------------
 Typedefs 
------------------------------------------------------------------------------*/

/**
  * @brief Standard status return codes for USB driver operations.
  */
typedef enum {
    USB_OK = 0,             /*!< Operation completed successfully              */
    USB_FAIL,               /*!< Hardware error, uninitialized, or unplugged   */
    USB_TIMEOUT,            /*!< Operation exceeded the specified time limit   */
    USB_DISCONNECTED        /*!< Cable physically disconnected mid-operation   */
} USB_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/**
  * @brief  Initializes the hardware for serial communication.
  * @retval USB_STATUS: USB_OK on success.
  */
USB_STATUS usb_init(void);
/**
  * @brief  Transmits a specified number of bytes over USB.
  * @param  tx_data_ptr:  Pointer to the data buffer to send.
  * @param  tx_data_size: Number of bytes to transmit.
  * @param  timeout:      Maximum time (in ms) to wait before giving up.
  * @retval USB_STATUS: USB_OK, USB_TIMEOUT, or USB_FAIL.
  */
USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout);

/**
  * @brief  Receives a specified number of bytes from the USB port.
  * @param  rx_data_ptr:  Pointer to the buffer where received data will be stored.
  * @param  rx_data_size: Exact number of bytes to wait for.
  * @param  timeout:      Maximum time (in ms) to wait before giving up.
  * @note   Timeout should be sized to accommodate the full expected byte count.
  * @retval USB_STATUS: USB_OK, USB_TIMEOUT, USB_FAIL, or USB_DISCONNECTED.
  */
USB_STATUS usb_receive(void* rx_data_ptr, size_t rx_data_size, uint32_t timeout);

/**
  * @brief  Removes garbage data by flushing the receive buffer.
  * @note   In CDC mode, optionally blocks until any pending TX completes.
  * @param  flush_tx: If true, waits for pending transmission to complete.
  *                   Parameter only present in CDC mode.
  * @retval USB_STATUS: USB_OK on success, USB_TIMEOUT if TX flush times out.
  *                     Legacy mode returns void.
  */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_flush(bool flush_tx);
#else
void usb_flush(void);
#endif /* USE_USB_CDC_FS */

#ifndef USE_USB_CDC_FS
/**
  * @brief  Receive a specified number of bytes over USB without blocking.
  * @note   Use an RX_CPLT callback to know when the received data is safe to use.
  * @param  tx_data_ptr: A pointer to the buffer to receive. Should be allocated
  *                      *statically* by the caller to prevent scoping issues.
  * @param  tx_data_size: The length of the buffer being received.
  * @retval USB_OK if everything is as expected, else an error that should be handled.
  */
USB_STATUS usb_receive_IT 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size  /* Size of the data to be received */
	);

/**
  * @brief  Transmit a specified number of bytes over USB without blocking.
  * @param  tx_data_ptr: A pointer to the buffer to transmit. Should be allocated
  *                      *statically* by the caller to prevent scoping issues.
  * @param  tx_data_size: The length of the buffer being transmitted.
  * @retval USB_OK if everything is as expected, else an error that should be handled.
  */
USB_STATUS usb_transmit_IT 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size  /* Size of transmit data */ 
	);
#endif

/**
  * @brief  Checks if the USB cable is physically connected (VBUS detected).
  * @note   Only available on hardware with a USB_DETECT GPIO. Calling this
  *         on an unsupported platform will result in a linker error.
  * @retval bool: true if connected, false if unplugged.
  */
#if defined( USE_USB_CDC_FS       ) || \
    defined( A0002_REV2           ) || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5           ) || \
    defined( L0005_REV3           )
bool usb_detect(void);
#endif /* USE_USB_CDC_FS || A0002_REV2 || FLIGHT_COMPUTER_LITE || L0002_REV5 || L0005_REV3 */


#ifdef __cplusplus
}
#endif

#endif /* USB_H */
