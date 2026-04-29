/**
  ******************************************************************************
  * @file           : usb_cdc.c
  * @brief          : Native USB CDC Serial Implementation
  * @author         : Sun Devil Rocketry Firmware Team
  *
  * @note  This file is conditionally included by usb.c when USE_USB_CDC_FS is
  *        defined. It provides the CDC-backed implementation of the USB driver
  *        API for the A0010 (Rev 3) board using the STM32 USB OTG peripheral.
  *
  *        Do NOT compile this file directly. It is included as a translation
  *        unit by usb.c based on the active build configuration.
  *
  *        Hardware target:
  *        - A0010 (Flight Computer Rev 3) → STM32H733, Native USB CDC
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
    if (CDC_Transmit_HS( tx_data_ptr, tx_data_size ) != USBD_OK){
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

} /* usb_transmit */


/* Receives a specified number of bytes from the USB port */
USB_STATUS usb_receive
    (
    void*    rx_data_ptr  ,
    size_t   rx_data_size ,
    uint32_t timeout
    )
{   /* ==================== API implementation =================== */
    
    if (!usb_detect() || rx_data_ptr == NULL || rx_data_size == 0){
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
        
        if (byte != -1){
            ((uint8_t*)rx_data_ptr)[count++] = (uint8_t)byte;
        }

        if ((HAL_GetTick() - tickstart) > timeout) {
            return USB_TIMEOUT;
        }
    }

    return USB_OK;

} /* usb_receive */


/* Detects an active USB connection via VBUS GPIO */
bool usb_detect( void )
{   /* ==================== API implementation =================== */
    /* Read voltage on USB detect pin and compare against GPIO Enum */
    if (HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_SET) {
        return true;
    } else {
        return false;
    }
} /* usb_detect */


/* Flushes pending data from the receive buffer, optionally waits for TX */
USB_STATUS usb_flush( bool flush_tx )
{   /* ==================== API implementation =================== */
    
    if (!usb_detect()) {
        return USB_FAIL;
    }

    /* Discard all pending bytes in receive buffer */
    uint32_t tickstart = HAL_GetTick();
    while (VCP_Bytes_Available() > 0) {
        VCP_Read_Byte();
        if ((HAL_GetTick() - tickstart) > USB_FLUSH_RX_TIMEOUT_MS){
            break;
        }
    }

    /* If requested, wait for pending transmission to complete */
    if (flush_tx){
        tickstart = HAL_GetTick();
        while (VCP_Is_Busy()) {
            if (!usb_detect() || (HAL_GetTick() - tickstart) > USB_FLUSH_TX_TIMEOUT_MS) {
                VCP_Force_Unlock();
                return USB_TIMEOUT;
            }
        }
    }

    return USB_OK;

} /* usb_flush */


/*******************************************************************************
* END OF usb_cdc.c                                                             *
*******************************************************************************/