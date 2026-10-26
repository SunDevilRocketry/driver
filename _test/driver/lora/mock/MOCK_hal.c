/*******************************************************************************
*
* FILE: 
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for HAL to trick
*      the tests into compiling.
*
*******************************************************************************/

#include <stdint.h>
#include "main.h"
#include "telemetry.h"
#include <string.h>
#include "usb.h"
HAL_StatusTypeDef mocked_return = HAL_OK; /* Default to "OK" return */

/* Buffers/state exposed to the test file */
uint8_t  mock_spi_tx_buffer[MOCK_SPI_BUFFER_SIZE];
uint16_t mock_spi_tx_size = 0;
uint8_t  mock_spi_tx_history[MOCK_SPI_BUFFER_SIZE];
uint16_t mock_spi_tx_history_size = 0;
uint8_t  mock_spi_rx_data[MOCK_SPI_BUFFER_SIZE];
uint16_t mock_spi_rx_offset = 0;
uint16_t mock_spi_tx_call_count = 0;
uint16_t mock_spi_rx_call_count = 0;
uint16_t mock_spi_fail_tx_call = 0;
uint16_t mock_spi_fail_rx_call = 0;
uint32_t mock_hal_tick = 0;
uint32_t mock_hal_tick_step = 0;
uint32_t mock_hal_delay_total = 0;
uint32_t mock_gpio_write_count = 0;
TELEMETRY_MESSAGE mock_telemetry_message;

/* Copies SPI data into the test-accessible buffer (truncated to buffer size) */
static void mock_spi_capture
    (
    const uint8_t* pData,
    uint16_t       size
    )
{
uint16_t n = ( size > MOCK_SPI_BUFFER_SIZE ) ? MOCK_SPI_BUFFER_SIZE : size;
if ( pData != NULL )
    {
    memcpy( mock_spi_tx_buffer, pData, n );
    uint16_t history_remaining = MOCK_SPI_BUFFER_SIZE - mock_spi_tx_history_size;
    if ( n > history_remaining )
        {
        n = history_remaining;
        }
    if ( n > 0 )
        {
        memcpy( &mock_spi_tx_history[mock_spi_tx_history_size], pData, n );
        mock_spi_tx_history_size += n;
        }
    }
mock_spi_tx_size = size;
}

void HAL_GPIO_WritePin
    (
    GPIO_TypeDef* GPIOx,
    uint16_t      GPIO_Pin,
    GPIO_PinState PinState
    )
{
mock_gpio_write_count++;
}

void HAL_Delay
    (
    uint32_t Delay
    )
{
mock_hal_delay_total += Delay;
}

uint32_t HAL_GetTick
    (
    void
    )
{
uint32_t tick = mock_hal_tick;
mock_hal_tick += mock_hal_tick_step;
return tick;
}

HAL_StatusTypeDef HAL_SPI_Transmit
    (
    SPI_HandleTypeDef* hspi,
    const uint8_t*     pData,
    uint16_t           Size,
    uint32_t           Timeout
    )
{
mock_spi_capture( pData, Size );
mock_spi_tx_call_count++;
if( mock_spi_tx_call_count == mock_spi_fail_tx_call )
    {
    return HAL_ERROR;
    }
return mocked_return;
}

HAL_StatusTypeDef HAL_SPI_Transmit_IT
    (
    SPI_HandleTypeDef* hspi,
    const uint8_t*     pData,
    uint16_t           Size
    )
{
mock_spi_capture( pData, Size );
return mocked_return;
}

HAL_StatusTypeDef HAL_SPI_Receive
    (
    SPI_HandleTypeDef* hspi,
    uint8_t*           pData,
    uint16_t           Size,
    uint32_t           Timeout
    )
{
uint16_t n = ( Size > MOCK_SPI_BUFFER_SIZE ) ? MOCK_SPI_BUFFER_SIZE : Size;
mock_spi_rx_call_count++;
if ( pData != NULL )
    {
    uint16_t available = 0;
    if ( mock_spi_rx_offset < MOCK_SPI_BUFFER_SIZE )
        {
        available = MOCK_SPI_BUFFER_SIZE - mock_spi_rx_offset;
        if ( available > n )
            {
            available = n;
            }
        memcpy( pData, &mock_spi_rx_data[mock_spi_rx_offset], available );
        }
    if ( available < n )
        {
        memset( &pData[available], 0, n - available );
        }
    }
mock_spi_rx_offset += Size;
if( mock_spi_rx_call_count == mock_spi_fail_rx_call )
    {
    return HAL_ERROR;
    }
return mocked_return;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT
    (
    SPI_HandleTypeDef* hspi,
    const uint8_t*     pTxData,
    uint8_t*           pRxData,
    uint16_t           Size
    )
{
mock_spi_capture( pTxData, Size );
if ( pRxData != NULL )
    {
    uint16_t n = ( Size > MOCK_SPI_BUFFER_SIZE ) ? MOCK_SPI_BUFFER_SIZE : Size;
    uint16_t available = 0;
    if ( mock_spi_rx_offset < MOCK_SPI_BUFFER_SIZE )
        {
        available = MOCK_SPI_BUFFER_SIZE - mock_spi_rx_offset;
        if ( available > n )
            {
            available = n;
            }
        memcpy( pRxData, &mock_spi_rx_data[mock_spi_rx_offset], available );
        }
    if ( available < n )
        {
        memset( &pRxData[available], 0, n - available );
        }
    mock_spi_rx_offset += Size;
    }
return mocked_return;
}

USB_STATUS usb_transmit
    (
    void*    tx_data_ptr,
    size_t   tx_data_size,
    uint32_t timeout
    )
{
return USB_OK;
}

USB_STATUS usb_receive
    (
    void*    rx_data_ptr,
    size_t   rx_data_size,
    uint32_t timeout
    )
{
return USB_OK;
}

void telemetry_get_next_message
    (
    TELEMETRY_MESSAGE* payload
    )
{
if ( payload != NULL )
    {
    *payload = mock_telemetry_message;
    }
}

/* Used to mock these functions */
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    )
{
mocked_return = statusToReturn;
}

HAL_StatusTypeDef HAL_UART_Receive_IT 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t Size
    )      
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Receive 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t size, 
    uint32_t timeout
    )      
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Transmit 
    (
    UART_HandleTypeDef *huart, 
    const uint8_t* pData, 
    uint16_t size, 
    uint32_t timeout
    )      
{
return mocked_return;
}
