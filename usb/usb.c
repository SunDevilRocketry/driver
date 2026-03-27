/**
  ******************************************************************************
  * @file           : usb.c
  * @brief          : Universal Serial Driver Interface
  * @author         : Sun Devil Rocketry Firmware Team
  * 
  * @note This driver supports both Native USB CDC (for A0010/Rev3+) and 
  *       Legacy UART-to-USB bridges via conditional compilation.
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
#elif defined( A0010 )
    #include "sdr_pin_defines_A0010.h"
#endif


/*------------------------------------------------------------------------------
 Standard Includes                                                               
------------------------------------------------------------------------------*/
#include "usb.h"
#include <string.h>
#include <stdbool.h>

/* Configuration Constants */
#define USB_FLUSH_RX_TIMEOUT_MS   100   // Timeout for flushing receive buffer
#define USB_FLUSH_TX_TIMEOUT_MS   1000  // Timeout for flushing transmit buffer
#define USB_VBUS_DISCHARGE_MS     100   // Wait time for VBUS discharge on deinit

/* ========================================================================== */
/* ======================== NATIVE USB CDC CONFIG  ========================== */
/* ========================================================================== */
#ifdef USE_USB_CDC_FS

#include "usb_device.h"
#include "usbd_cdc_if.h"

// Middleware handle for state control
extern USBD_HandleTypeDef hUsbDeviceHS;

/* 
 * VBUS Sensing Macro. 
 * Maps to USB_DETECT as per SDR hardware schematic.
 */
#define IS_USB_CONNECTED() (HAL_GPIO_ReadPin(USB_DETECT_GPIO_PORT, USB_DETECT_PIN) == GPIO_PIN_SET)

/* 
 * Helpers implemented in usbd_cdc_if.c from usbd_cdc_if.h
 * These allow us to bypass the standard non-blocking CDC callbacks 
 * and force a blocking-esk design.
 */
int16_t  VCP_Read_Byte(void);
uint8_t  VCP_Is_Busy(void);
uint16_t VCP_Bytes_Available(void); 
void VCP_Force_Unlock(void);

/* --- Functions --- */

USB_STATUS usb_init(void) 
{
    MX_USB_DEVICE_Init(); // Initialize the ST USB Middleware stack
    return USB_OK;
}

void usb_poll_connection(void) 
{
    static uint8_t previous_vbus_state = 2; // 2 = uninitialized
    uint8_t current_vbus_state = IS_USB_CONNECTED() ? 1 : 0;

    // Handle state transitions
    if (current_vbus_state != previous_vbus_state) {
        if (current_vbus_state == 1) {
            USBD_Start(&hUsbDeviceHS);
        } 
        else if (previous_vbus_state != 2) {
            USBD_Stop(&hUsbDeviceHS);
            
            // Clear dangling transmit lock in case
            VCP_Force_Unlock();
        }
        previous_vbus_state = current_vbus_state;
    }
}

USB_STATUS usb_transmit(uint8_t* data, uint16_t size, uint32_t timeout_ms) 
{
    if (data == NULL || size == 0 || !IS_USB_CONNECTED()) {
        return USB_FAIL;
    }

    uint32_t tickstart = HAL_GetTick();
    
    /* 1. Wait for any ongoing transmission to complete */
    while (VCP_Is_Busy()) {
        if (!IS_USB_CONNECTED() || (HAL_GetTick() - tickstart) > timeout_ms) {
            
            // Force 'unlock' from ST middleware to prevent 'deadlock'
            VCP_Force_Unlock();
            return USB_TIMEOUT;
        }
    }
    
    /* 2. Submit the buffer to the USB hardware */
    if (CDC_Transmit_HS(data, size) != USBD_OK) {
        return USB_FAIL;
    }
    
    /* 3. Block until the hardware finishes pushing the data */
    while (VCP_Is_Busy()) {
        if (!IS_USB_CONNECTED() || (HAL_GetTick() - tickstart) > timeout_ms) {
            
            // Force 'unlock' from ST middleware to prevent 'deadlock'
            VCP_Force_Unlock();
            return USB_TIMEOUT;
        }
    }
    
    return USB_OK; 
}

USB_STATUS usb_receive(uint8_t* data, uint16_t size, uint32_t timeout_ms) 
{
    if (!IS_USB_CONNECTED() || data == NULL || size == 0) {
        return USB_FAIL;
    }

    uint32_t tickstart = HAL_GetTick();
    uint16_t count = 0;

    /* Poll the buffer until requested byte count is met */
    while (count < size) {

        if (!IS_USB_CONNECTED()) {
            return USB_FAIL;
        }

        int16_t byte = VCP_Read_Byte();
        
        if (byte != -1) {
            data[count++] = (uint8_t)byte;
        }
        
        if ((HAL_GetTick() - tickstart) > timeout_ms) {
            return USB_TIMEOUT;
        }
    }
    
    return USB_OK;
}

uint8_t usb_data_available(void) 
{
    if (!IS_USB_CONNECTED()) {
        return 0;
    }

    return (VCP_Bytes_Available() > 0) ? 1 : 0;
}


bool usb_detect(void) 
{
    return IS_USB_CONNECTED();
}


USB_STATUS usb_deinit(void) 
{
    if (hUsbDeviceHS.pClassData != NULL) {
        // Force unlock to clear any stuck transmission
        VCP_Force_Unlock();
        
        // Call the deinit function
        MX_USB_DEVICE_DeInit();

        // Clear receive buffer
        VCP_Clear_RxBuffer();
        
        // Wait for VBUS to discharge
        HAL_Delay(USB_VBUS_DISCHARGE_MS);
    }
    
    return USB_OK;
}

USB_STATUS usb_flush(bool flush_tx) 
{
    if (!IS_USB_CONNECTED()) {
        return USB_FAIL;
    }
    
    // 1. Clear receive buffer
    uint32_t tickstart = HAL_GetTick();
    while (VCP_Bytes_Available() > 0) {
        // Discard bytes
        VCP_Read_Byte();
        
        // Timeout protection
        if ((HAL_GetTick() - tickstart) > USB_FLUSH_RX_TIMEOUT_MS) {
            break;
        }
    }
    
    // 2. (If choosen) Wait for pending transmission to complete
    if (flush_tx) {
        tickstart = HAL_GetTick();
        while (VCP_Is_Busy()) {
            if (!IS_USB_CONNECTED() || (HAL_GetTick() - tickstart) > USB_FLUSH_TX_TIMEOUT_MS) {
                VCP_Force_Unlock();
                return USB_TIMEOUT;
            }
        }
    }
    
    return USB_OK;
}
#else

/* ========================================================================== */
/* ========================== LEGACY UART CONFIG ============================ */
/* ========================================================================== */

/* --- Private Variables --- */
static UART_HandleTypeDef* g_huart = NULL;

/* --- Functions --- */

USB_STATUS usb_init(UART_HandleTypeDef* huart) 
{
    if (huart == NULL) {
        return USB_FAIL;
    }
    g_huart = huart;
    return USB_OK;
}

void usb_poll_connection(void){ /* Dummy Function O.O */}

USB_STATUS usb_transmit(uint8_t* data, uint16_t size, uint32_t timeout_ms) 
{
    if (g_huart == NULL || data == NULL || size == 0) {
        return USB_FAIL;
    }
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(g_huart, data, size, timeout_ms);
    
    if (status == HAL_OK)      return USB_OK;
    if (status == HAL_TIMEOUT) return USB_TIMEOUT;
    
    return USB_FAIL;
}

USB_STATUS usb_receive(uint8_t* data, uint16_t size, uint32_t timeout_ms) 
{
    if (g_huart == NULL || data == NULL || size == 0) {
        return USB_FAIL;
    }
    
    HAL_StatusTypeDef status = HAL_UART_Receive(g_huart, data, size, timeout_ms);

    if (status == HAL_OK)      return USB_OK;
    if (status == HAL_TIMEOUT) return USB_TIMEOUT;
    
    return USB_FAIL;
}

uint8_t usb_data_available(void) 
{
    if (g_huart == NULL) {
        return 0;
    }
    
    /* Check hardware register directly to see if a byte is sitting in the RX line */
    return (__HAL_UART_GET_FLAG(g_huart, UART_FLAG_RXNE) ? 1 : 0);
}

USB_STATUS usb_deinit(void) 
{
    if (g_huart != NULL) {
        HAL_UART_Abort(g_huart);
        g_huart = NULL;
    }
    return USB_OK;
}

USB_STATUS usb_flush(bool flush_tx) 
{
    if (g_huart == NULL) {
        return USB_FAIL;
    }
    
    // Clear UART receive buffer (if any)
    if (g_huart != NULL && g_huart->Instance != NULL) {
        while (__HAL_UART_GET_FLAG(g_huart, UART_FLAG_RXNE)) {
            volatile uint8_t dummy = g_huart->Instance->RDR; // May be called DR on STM32F series for instance
            (void)dummy;
        }
    }
    
    // Wait for transmission if requested
    if (flush_tx && g_huart != NULL) {
        uint32_t tickstart = HAL_GetTick();
        while (!__HAL_UART_GET_FLAG(g_huart, UART_FLAG_TC)) {
            if ((HAL_GetTick() - tickstart) > USB_FLUSH_TX_TIMEOUT_MS) {
                return USB_TIMEOUT;
            }
        }
    }
    
    return USB_OK;
}


#endif
