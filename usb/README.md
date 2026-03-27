# Unified USB/Serial Driver Module

The USB driver (`usb.c`/`usb.h`) provides an abstraction layer for serial terminal communication. It automatically detects hardware configuration via compiler flags to switch between **Native STM32 USB CDC (Virtual COM Port)** or **Legacy UART**.

### Features
- **Consistent API**: Use the same function calls regardless of whether the board uses a USB-UART bridge or internal USB.
- **State Connection**: Automatically detects cable disconnects/reconnects and resets the internal USB state machine to prevent terminal hangs.
- **Blocking Design**: Designed for synchronization reliability and "blocking-esk" behavior in a non-blocking middleware environment.
- **Internal Buffering**: Utilizes a circular "storage tank" for USB reception to prevent data loss during terminal command entry.

---

### Hardware Assumptions

#### 1. Target Hardware
- **Primary**: STM32H753 or STM32H733 (Rev 3 PCB / A0010).
- **Legacy**: Any STM32 H7/L4 board using a standard UART interface (e.g., A0002, L0002, L0005).

#### 2. Configuration (CubeMX / Middleware)
To function correctly, the following hardware configurations are assumed:
- **For Native USB (`USE_USB_CDC_FS`)**: 
    - `USB_OTG_HS` enabled as **Internal FS Phy** (Device Only).
    - `USB_DEVICE` Middleware set to **Communication Device Class (CDC)**.
    - **Clock**: The USB clock must be exactly **48 MHz**.
    - **VBUS Sensing**: Handled manually via the `USB_DETECT` pin (typically PA10).
    - **USB_DETECT**: Configured as a **GPIO Input**. It is highly recommended to have an **Internal Pull-Down** enabled to prevent floating signal noise when unplugged.
- **For Legacy (UART)**:
    - Standard UART TX/RX pins configured.
    - **Baud Rate**: Typically 115200.

---

### Build-Time Selection
The driver switches modes using the `USE_USB_CDC_FS` definition (set via Makefile or Compiler Symbols). 

- **Define Set**: Driver compiles for **Native USB CDC** .
- **Define Unset**: Driver compiles for **UART** (Legacy).

---

### API Reference

#### Initialization
```c
/* If USE_USB_CDC_FS is defined */
USB_STATUS usb_init(void);

/* If USE_USB_CDC_FS is NOT defined */
USB_STATUS usb_init(UART_HandleTypeDef* huart);
```

#### Connection Management
```c
void usb_poll_connection(void);
```
**Note:** This must be called periodically in the main loop. It monitors the physical VBUS connection (`USB_DETECT`), starting or stopping the USB device stack if the cable is hot-swapped.

#### Primary Communication (Blocking)
These functions block execution until the operation is complete or the timeout is reached.
```c
USB_STATUS usb_transmit(uint8_t* data, uint16_t size, uint32_t timeout_ms);
USB_STATUS usb_receive(uint8_t* data, uint16_t size, uint32_t timeout_ms);
```

| Function | Description |
| :--- | :--- |
| `usb_transmit` | Sends data; waits until the hardware finishes pushing the data or timeout. |
| `usb_receive` | Polls the internal buffer until *exactly* the requested number of bytes are gathered. |

#### Helpers
```c
uint8_t usb_data_available(void);
bool    usb_detect(void);
```
| Function | Description |
| :--- | :--- |
| `usb_data_available` | Returns 1 if there is unread data waiting in the internal circular buffer. |
| `usb_detect` | Returns true if VBUS is physically detected (cable is plugged in). |

---

### Status Codes (`USB_STATUS`)
- `USB_OK` : Operation successful.
- `USB_FAIL`: Hardware is uninitialized, physically unplugged, or a driver error occurred.
- `USB_TIMEOUT`: The operation took longer than the allocated `timeout_ms`.

---

### Usage Example (Terminal Echo)

This code works on Native USB and Legacy boards without changing the application logic.

```c
#include "usb.h"
#include <string.h>

int main(void) {
    // 1. Initialize based on configuration
    #ifdef USE_USB_CDC_FS
        usb_init();
    #else
        usb_init(&huart3); // Pass your specific UART handle here
    #endif

    char* welcome = "Hiii UwU\r\n";
    usb_transmit((uint8_t*)welcome, strlen(welcome), 100);

    uint8_t rx_byte;

    while(1) {
        // 2. Poll for connection state (Handles hardware Start/Stop)
        usb_poll_connection();

        // 3. Check for available data
        if (usb_data_available()) {
            // 4. Blocking read of 1 byte
            if (usb_receive(&rx_byte, 1, 10) == USB_OK) {
                // 5. Echo back to terminal
                usb_transmit(&rx_byte, 1, 10);
            }
        }
    }
}
```

---

### Implementation Details:
The core of making the asynchronous ST USB Middleware behave as a blocking driver is the use of a circular "storage tank" and a force-unlock mechanism.

- **Circular Buffer**: Located in `usbd_cdc_if.c`, this 2048-byte buffer captures all incoming data from the `CDC_Receive_HS` interrupt, allowing the main loop to read it at its own pace.
- **Deadlock Prevention**: If a transmission is interrupted by a physical cable disconnect, the driver utilizes `VCP_Force_Unlock()` to reset the `TxState` of the USB peripheral, preventing the firmware from hanging on the next attempt.
