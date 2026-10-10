/*******************************************************************************
*
* FILE: 
*      main.h (MOCK)
*
* DESCRIPTION: 
*      Header file to trick the test into compiling. Also contains mock function
*      definitions.
*
*******************************************************************************/

#include "pindefs.h"
#include "stm32h7xx_hal.h"

#define HAL_DEFAULT_TIMEOUT 10u
#define MOCK_SPI_BUFFER_SIZE 128u

/* Mock state accessible from tests (defined in MOCK_hal.c) */
extern uint8_t  mock_spi_tx_buffer[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_tx_size;
extern uint8_t  mock_spi_tx_history[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_tx_history_size;
extern uint8_t  mock_spi_rx_data[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_rx_offset;
extern uint16_t mock_spi_tx_call_count;
extern uint16_t mock_spi_rx_call_count;
extern uint16_t mock_spi_fail_tx_call;
extern uint16_t mock_spi_fail_rx_call;
extern uint32_t mock_hal_tick;
extern uint32_t mock_hal_tick_step;
extern uint32_t mock_hal_delay_total;
extern uint32_t mock_gpio_write_count;

void reset_test
    (
    void
    );

void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    );