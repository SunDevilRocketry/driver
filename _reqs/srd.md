<!-- next traceable tag: RQ.DRIVER.00000 -->
<!-- This req doc is written and updated manually and checked in along with source code -->
<!-- Desired behaviors are expected to be specified by human engineers. No AI assistance allowed for requirement specification -->
# SDR "driver" Library Software Requirements Document

### QA level: Safety Critical
### Part Number: A0016-XXX

## 1. Structural

RQ.DRIVER.00001 - The library shall be organized into self-contained modules with header files organized next to source files.

RQ.DRIVER.00002 - The library shall be designed to be compiled with GNU GCC for the desired target architecture.

### 1.1. Dependencies

RQ.DRIVER.00003 - The library shall be designed to interface with the Sun Devil Rocketry "mod" library.

    > Test plan: For the purposes of verifying the library, tests will assume these modules are used on A0002-2XX Flight Computer. This does not require the flight computer to release at the same time, however library integration is tested via the flight computer. Thus, the required coverage burden will only be achieved if the flight computer provides much of it.

RQ.DRIVER.00004 - The library shall be designed to interface with the STMicroelectronics HAL Library for H7 series microcontrollers.

RQ.DRIVER.00005 - The library shall be designed to consume the STMicroelectronics STM32_USB_Device_Library.

RQ.DRIVER.00006 - The library shall be designed to consume the LSM6DSV320x Platform Independent IMU Driver.

## 2. Functional

### 2.1. Baro

### 2.2. Buzzer

### 2.3. Camera

This driver was requested for a program that never used it. It has been left for future reference, but is not maintained by SDR.

### 2.4. Flash

### 2.5. GPS

### 2.6. Ignition

### 2.7. IMU

### 2.8. LED

### 2.9. Load Cell

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.10. LoRa

RQ.DRIVER.00007 - The library shall provide a method to send data via a RFM95 LoRa modem.

RQ.DRIVER.00008 - The library shall provide a method to receive data via a RFM95 LoRa modem.

RQ.DRIVER.00009 - The library shall provide a method to configure wireless transmission parameters.

RQ.DRIVER.00010 - The library shall provide a method to transmit data without blocking the CPU.

### 2.11. Onboard Flash

### 2.12. Power

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.13. Pressure

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.14. RS485

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.15. Servo

### 2.16. Solenoid

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.17. Temp

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.18. Timer

### 2.19. USB

### 2.20. Valve

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.

### 2.21. Wireless

This driver was written for a legacy piece of hardware and is no longer in use. The driver has been left for future reference, but is not maintained by SDR.
