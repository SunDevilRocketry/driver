/**
  ******************************************************************************
  * @file           : usb.h
  * @brief          : Universal Serial Communication Driver
  * @author         : Sun Devil Rocketry Firmware Team
  * 
  * @note           : Provides a unified interface for both Native USB CDC 
  *                   (A0010) and Legacy UART-to-USB bridges. 
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
#include "main.h"       /* Required for HAL types */
#include <stdint.h>     
#include <stddef.h> 
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/**
  * @brief Standard status return codes for USB driver operations.
  */
typedef enum {
    USB_OK = 0,             /*!< Operation completed successfully */
    USB_FAIL,               /*!< Hardware error or invalid parameters */
    USB_TIMEOUT             /*!< Operation exceeded the specified time limit */
} USB_STATUS;


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes the hardware for serial communication.
  * @note   For A0010 (USE_USB_CDC_FS), this starts the USB middleware.
  *         For legacy boards, it assigns the UART handle.
  * 
  * @param  huart: Pointer to the UART handle (Only used for legacy UART mode).
  * @retval USB_STATUS: USB_OK on success.
  */
#ifdef USE_USB_CDC_FS
USB_STATUS usb_init(void);
#else
USB_STATUS usb_init(UART_HandleTypeDef* huart);
#endif

/**
  * @brief  Monitors the physical connection state of the USB cable.
  * @note   Must be called periodically (e.g., in the main while loop). 
  *         Handles enumeration/disconnection syncing for USE_USB_CDC_FS. Does nothing for legacy.
  */
void usb_poll_connection(void);

/**
  * @brief  Transmits a buffer of data over the serial interface.
  * @param  data: Pointer to the byte array to send.
  * @param  size: Number of bytes to transmit.
  * @param  timeout_ms: Maximum time (in ms) to wait before giving up.
  * @retval USB_STATUS: USB_OK, USB_TIMEOUT, or USB_FAIL.
  */
USB_STATUS usb_transmit(uint8_t* data, uint16_t size, uint32_t timeout_ms);

/**
  * @brief  Receives a specific amount of data from the serial interface.
  * @param  data: Pointer to the buffer where received data will be stored.
  * @param  size: Exact number of bytes to wait for.
  * @param  timeout_ms: Maximum time (in ms) to wait before giving up.
  * @retval USB_STATUS: USB_OK, USB_TIMEOUT, or USB_FAIL.
  */
USB_STATUS usb_receive(uint8_t* data, uint16_t size, uint32_t timeout_ms);

/**
  * @brief  Checks if there is unread data waiting in the receive buffer.
  * @retval uint8_t: 1 if data is available, 0 if empty.
  */
uint8_t usb_data_available(void);

/**
 * @brief Deinitializes USB hardware and releases resources
 * @note For Native USB CDC mode, stops the USB device and forces disconnect
 *       For legacy mode, releases the UART handle reference
 */
USB_STATUS usb_deinit(void);

/**
 * @brief  Clears all pending data from receive buffer.
 * @param  flush_tx: If true, also waits for pending transmissions to complete
 * @retval USB_STATUS: USB_OK on success, USB_TIMEOUT if TX flush times out
 */
USB_STATUS usb_flush(bool flush_tx);

/* Checks for an active USB connection */
#if defined( USE_USB_CDC_FS ) || \
    defined( A0002_REV2 )     || \
    defined( FLIGHT_COMPUTER_LITE ) || \
    defined( L0002_REV5 )     || \
    defined( L0005_REV3 )
/**
  * @brief  Checks if the USB cable is physically connected (VBUS detected).
  * @retval bool: true if connected, false if unplugged.
  */
bool usb_detect(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* USB_H */
