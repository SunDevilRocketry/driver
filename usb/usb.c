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

/* Validates UART handle for legacy UART-to-USB bridge boards */
#ifndef USE_USB_CDC_FS
USB_STATUS usb_init( UART_HandleTypeDef* huart )
{   /* ==================== API implementation =================== */

    /* Null handle would cause silent dereference failures at runtime */
    if (huart == NULL) {
        return USB_FAIL;
    }
    return USB_OK;

} /* usb_init */
#endif /* USE_USB_CDC_FS */


/*------------------------------------------------------------------------------
 Build Configuration Notice

 Include the appropriate implementation source based on build configuration.
 These files are >not compiled< independently.
------------------------------------------------------------------------------*/
#ifdef USE_USB_CDC_FS
    #include "usb_cdc.c"
#else
    #include "usb_uart.c"
#endif /* USE_USB_CDC_FS */


/*******************************************************************************
* END OF usb.c                                                                 *
*******************************************************************************/