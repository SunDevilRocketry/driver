/*******************************************************************************
*
* FILE: 
*      test_gps.c
*
* DESCRIPTION: 
*      Unit tests for functions in the gps module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "lora.h"
#include "math_sdr.h"

// Allow mutation of static memory by including source files directly
#include "lora.c"


/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
SPI_HandleTypeDef hspi4;  /* LoRa SPI */

extern uint8_t mock_spi_tx_buffer[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_tx_size;
extern uint8_t mock_spi_tx_history[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_tx_history_size;
extern uint8_t mock_spi_rx_data[MOCK_SPI_BUFFER_SIZE];
extern uint16_t mock_spi_rx_offset;
extern uint16_t mock_spi_tx_call_count;
extern uint16_t mock_spi_rx_call_count;
extern uint16_t mock_spi_fail_tx_call;
extern uint16_t mock_spi_fail_rx_call;
extern HAL_StatusTypeDef mocked_return;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

static bool mock_spi_history_contains
	(
	const uint8_t* expected,
	uint16_t       expected_size
	)
{
if( expected_size > mock_spi_tx_history_size )
	{
	return false;
	}

for( uint16_t i = 0; i <= mock_spi_tx_history_size - expected_size; i++ )
	{
	if( memcmp( &mock_spi_tx_history[i], expected, expected_size ) == 0 )
		{
		return true;
		}
	}

return false;
}

/* Resets all mock state to defaults */
void reset_test
    (
    void
    )
{
mocked_return = HAL_OK;
memset( mock_spi_tx_buffer, 0, sizeof( mock_spi_tx_buffer ) );
memset( mock_spi_tx_history, 0, sizeof( mock_spi_tx_history ) );
memset( mock_spi_rx_data, 0, sizeof( mock_spi_rx_data ) );
mock_spi_tx_size = 0;
mock_spi_tx_history_size = 0;
mock_spi_rx_offset = 0;
mock_spi_tx_call_count = 0;
mock_spi_rx_call_count = 0;
mock_spi_fail_tx_call = 0;
mock_spi_fail_rx_call = 0;
mock_hal_tick = 0;
mock_hal_tick_step = 0;
mock_hal_delay_total = 0;
mock_gpio_write_count = 0;
is_lora_configured = false;
}


/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/


/**
 * @brief Tests LoRa initialization and configuration.
 */
void test_lora_init_configure
	(
	void
	)
{
LORA_CONFIG config =
	{
		.lora_mode = LORA_STANDBY_MODE,
		.lora_spread = LORA_SPREAD_7,
		.lora_bandwidth = LORA_BANDWIDTH_62_5_KHZ,
		.lora_ecr = LORA_ECR_4_5,
		.lora_header_mode = LORA_EXPLICIT_HEADER,
		.lora_pa_select = LORA_RFO,
		.lora_frequency = 915000
	};

TEST_begin_nested_case( "LoRa starts uninitialized" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "The driver is initially uninitialized", lora_is_lora_initialized(), false );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Initialization rejects an incorrect device ID" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x11;
	TEST_ASSERT_EQ_UINT( "Wrong chip ID is rejected", lora_init( &config ), LORA_FAIL );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Initialization rejects an unsupported bandwidth" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	config.lora_bandwidth = (LORA_BANDWIDTH)0xFF;
	TEST_ASSERT_EQ_UINT( "Unknown bandwidth is rejected", lora_init( &config ), LORA_FAIL );
	config.lora_bandwidth = LORA_BANDWIDTH_125_KHZ;
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Initialization rejects frequencies outside the ISM band" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	config.lora_frequency = ISM_MIN_FREQ - 1;
	TEST_ASSERT_EQ_UINT( "Out-of-band frequency is rejected", lora_init( &config ), LORA_FAIL );
	config.lora_frequency = 915000;
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Initialization accepts a valid configuration" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	TEST_ASSERT_EQ_UINT( "Valid configuration initializes the modem", lora_init( &config ), LORA_OK );
	TEST_end_nested_case();
	}

	struct SPI_FAILURE_CASE
		{
		const char* description;
		uint16_t transmit_call;
		uint16_t receive_call;
		};
	struct SPI_FAILURE_CASE failures[] =
		{
			{ "Sleep-mode opmode read command", 2, 0 },
			{ "Sleep-mode opmode read data", 0, 2 },
			{ "Sleep-mode opmode write", 3, 0 },
			{ "LoRa-mode opmode read command", 4, 0 },
			{ "LoRa-mode opmode read data", 0, 3 },
			{ "LoRa-mode opmode write", 5, 0 },
			{ "Spread-factor read command", 6, 0 },
			{ "Spread-factor read data", 0, 4 },
			{ "Spread-factor write", 7, 0 },
			{ "Bandwidth read command", 8, 0 },
			{ "Bandwidth read data", 0, 5 },
			{ "Bandwidth write", 9, 0 },
			{ "Frequency MSB write", 10, 0 },
			{ "Frequency middle-byte write", 11, 0 },
			{ "Frequency LSB write", 12, 0 },
			{ "PA configuration write", 13, 0 },
			{ "Standby opmode read command", 14, 0 },
			{ "Standby opmode read data", 0, 6 },
			{ "Standby opmode write", 15, 0 }
		};

	for( int i = 0; i < array_size( failures ); i++ )
		{
		TEST_begin_nested_case( failures[i].description );
			{
			reset_test();
			mock_spi_rx_data[0] = 0x12; /* Device ID read succeeds */
			mock_spi_fail_tx_call = failures[i].transmit_call;
			mock_spi_fail_rx_call = failures[i].receive_call;
			TEST_ASSERT_EQ_UINT( "Initialization fails after the injected register I/O error", lora_init( &config ), LORA_FAIL );
			TEST_end_nested_case();
			}
		}

TEST_begin_nested_case( "Configure applies defaults and marks the driver initialized when preset is NULL" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	TEST_ASSERT_EQ_UINT( "Default configuration is reported", lora_configure( NULL ), LORA_USING_DEFAULTS );
	TEST_ASSERT_EQ_UINT( "Driver is marked initialized", lora_is_lora_initialized(), true );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Configure applies defaults and marks the driver initialized when preset is 0x00" );
	{
	LORA_PRESET test_preset;
	memset( &test_preset, 0x00, sizeof(LORA_PRESET) );
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	TEST_ASSERT_EQ_UINT( "Default configuration is reported", lora_configure( &test_preset ), LORA_USING_DEFAULTS );
	TEST_ASSERT_EQ_UINT( "Driver is marked initialized", lora_is_lora_initialized(), true );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Configure applies defaults and marks the driver initialized when preset is 0xFF" );
	{
	LORA_PRESET test_preset;
	memset( &test_preset, 0xFF, sizeof(LORA_PRESET) );
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	TEST_ASSERT_EQ_UINT( "Default configuration is reported", lora_configure( &test_preset ), LORA_USING_DEFAULTS );
	TEST_ASSERT_EQ_UINT( "Driver is marked initialized", lora_is_lora_initialized(), true );
	TEST_end_nested_case();
	}

for( int i = 0; i < LORA_BANDWIDTH_500_KHZ + 1; i++ )
	{
	char str_buf[128];
	snprintf( str_buf, 128, "Configure accepts a valid preset (bandwidth: %d)", i );
	TEST_begin_nested_case( str_buf );
		{
		LORA_PRESET preset =
			{
				.lora_spread = LORA_SPREAD_7,
				.lora_bandwidth = i,
				.lora_ecr = 5,
				.high_power_mode = false,
				.lora_frequency = 915000
			};
		reset_test();
		mock_spi_rx_data[0] = 0x12;
		TEST_ASSERT_EQ_UINT( "Valid preset is accepted", lora_configure( &preset ), LORA_OK );
		TEST_ASSERT_EQ_UINT( "Configured driver remains initialized", lora_is_lora_initialized(), true );
		TEST_end_nested_case();
		}
	}

TEST_begin_nested_case( "Configure reports fail on bad chip ID" );
	{
	LORA_PRESET preset =
		{
			.lora_spread = LORA_SPREAD_7,
			.lora_bandwidth = LORA_BANDWIDTH_125_KHZ,
			.lora_ecr = 5,
			.high_power_mode = false,
			.lora_frequency = 915000
		};
	reset_test();
	mock_spi_rx_data[0] = 0x11; /* Incorrect device ID */
	TEST_ASSERT_EQ_UINT( "Bad initialization returns a fail status", lora_configure( &preset ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Bad initialization marks the driver as not configured", lora_is_lora_initialized(), false );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Configure reports fail on bad read during init" );
	{
	LORA_PRESET preset =
		{
			.lora_spread = LORA_SPREAD_7,
			.lora_bandwidth = LORA_BANDWIDTH_500_KHZ,
			.lora_ecr = 5,
			.high_power_mode = false,
			.lora_frequency = 915000
		};
	reset_test();
	mock_spi_rx_data[0] = 0x12;
	mock_spi_fail_tx_call = 3; /* Fail the first register write after the ID read */
	TEST_ASSERT_EQ_UINT( "Bad initialization returns a fail status", lora_configure( &preset ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Bad initialization marks the driver as not configured", lora_is_lora_initialized(), false );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests changing the LoRa chip operation mode.
 */
void test_lora_set_chip_mode
	(
	void
	)
{
TEST_begin_nested_case( "Set chip mode preserves unrelated operation-mode bits" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0xA0;
	TEST_ASSERT_EQ_UINT( "Mode update succeeds", lora_set_chip_mode( LORA_RX_CONTINUOUS_MODE ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Writes operation-mode register", mock_spi_tx_buffer[0], LORA_REG_OPERATION_MODE | 0x80 );
	TEST_ASSERT_EQ_UINT( "Preserves upper bits and sets RX mode", mock_spi_tx_buffer[1], 0xA5 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Set chip mode reports an SPI failure" );
	{
	reset_test();
	MOCK_HAL_Status_Return( HAL_ERROR );
	TEST_ASSERT_EQ_UINT( "Mode update fails when SPI fails", lora_set_chip_mode( LORA_STANDBY_MODE ), LORA_FAIL );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests the LoRa hardware reset sequence.
 */
void test_lora_reset
	(
	void
	)
{
TEST_begin_nested_case( "Reset pulses the reset pin and waits 20 ms" );
	{
	reset_test();
	lora_reset();
	TEST_ASSERT_EQ_UINT( "Reset drives GPIO twice", mock_gpio_write_count, 2 );
	TEST_ASSERT_EQ_UINT( "Reset waits for 20 ms total", mock_hal_delay_total, 20 );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests the DMA transmit lifecycle: start, key-up, TxDone, error.
 */
void test_lora_transmit_async
	(
	void
	)
{
uint8_t payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
uint8_t dio0_txdone[2] = { LORA_REG_DIO_MAPPING_MODE_1 | 0x80, 0x40 };
uint8_t dio0_rxdone[2] = { LORA_REG_DIO_MAPPING_MODE_1 | 0x80, 0x00 };
uint8_t tx_mode[2]     = { LORA_REG_OPERATION_MODE | 0x80, 0x80 | LORA_TRANSMIT_MODE };

TEST_begin_nested_case( "Transmit starts, remaps DIO0 to TxDone, and locks the bus" );
	{
	reset_test();
	dma_busy = false;
	dma_awaiting_txdone = false;
	mock_spi_rx_data[0] = 0x80 | LORA_SLEEP_MODE; /* RegOpMode with the LoRa bit set */
	TEST_ASSERT_EQ_UINT( "Transmit starts", lora_transmit_async( payload, sizeof( payload ) ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "DIO0 is mapped to TxDone", mock_spi_history_contains( dio0_txdone, 2 ), true );
	TEST_ASSERT_EQ_MEMORY( "Payload is handed to DMA", mock_spi_tx_buffer, payload, sizeof( payload ) );
	TEST_ASSERT_EQ_UINT( "Second transmit is rejected while busy", lora_transmit_async( payload, sizeof( payload ) ), LORA_BUSY );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "DMA complete keys up TX; TxDone releases the bus and restores DIO0" );
	{
	mock_spi_tx_history_size = 0;
	lora_process_async_cb();
	TEST_ASSERT_EQ_UINT( "Transmit mode is written", mock_spi_history_contains( tx_mode, 2 ), true );
	TEST_ASSERT_EQ_UINT( "Still busy until TxDone", lora_transmit_async( payload, sizeof( payload ) ), LORA_BUSY );
	lora_process_dio0_cb();
	TEST_ASSERT_EQ_UINT( "DIO0 is mapped back to RxDone", mock_spi_history_contains( dio0_rxdone, 2 ), true );
	TEST_ASSERT_EQ_UINT( "Busy flag is released", dma_busy, false );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "A missed TxDone times out, clears the IRQ flags, and frees the bus" );
	{
	uint8_t clear_irq[2] = { LORA_REG_IRQ_FLAGS | 0x80, 0xFF };
	reset_test();
	dma_busy = false;
	dma_awaiting_txdone = false;
	mock_spi_rx_data[0] = 0x80 | LORA_SLEEP_MODE;
	lora_transmit_async( payload, sizeof( payload ) );
	lora_process_async_cb(); /* keyed up, TxDone never arrives */
	mock_hal_tick = txdone_deadline;
	TEST_ASSERT_EQ_UINT( "Still busy at the deadline", lora_transmit_async( payload, sizeof( payload ) ), LORA_BUSY );
	mock_hal_tick = txdone_deadline + 1;
	mock_spi_tx_history_size = 0;
	TEST_ASSERT_EQ_UINT( "Past the deadline reports a timeout", lora_transmit_async( payload, sizeof( payload ) ), LORA_TIMEOUT_FAIL );
	TEST_ASSERT_EQ_UINT( "Stuck IRQ flags are cleared", mock_spi_history_contains( clear_irq, 2 ), true );
	TEST_ASSERT_EQ_UINT( "Busy flag is released", dma_busy, false );
	mock_spi_rx_offset = 0;
	TEST_ASSERT_EQ_UINT( "The next transmit starts", lora_transmit_async( payload, sizeof( payload ) ), LORA_OK );
	lora_process_async_error_cb();
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "A stalled TX DMA is aborted and the bus freed" );
	{
	extern uint16_t mock_spi_abort_calls;
	reset_test();
	mock_spi_abort_calls = 0;
	dma_busy = false;
	dma_awaiting_txdone = false;
	mock_spi_rx_data[0] = 0x80 | LORA_SLEEP_MODE;
	lora_transmit_async( payload, sizeof( payload ) ); /* DMA complete never arrives */
	mock_hal_tick = txdone_deadline + 1;
	mock_gpio_write_count = 0;
	TEST_ASSERT_EQ_UINT( "Past the deadline reports a timeout", lora_transmit_async( payload, sizeof( payload ) ), LORA_TIMEOUT_FAIL );
	TEST_ASSERT_EQ_UINT( "The SPI DMA is aborted", mock_spi_abort_calls, 1 );
	TEST_ASSERT_EQ_UINT( "No DMA is left pending", dma_pending_op, LORA_DMA_OP_NONE );
	TEST_ASSERT_EQ_UINT( "Busy flag is released", dma_busy, false );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "TxDone timeout covers time on air with margin" );
	{
	symbol_time_us = 32768U; /* SF12 @ 125 kHz */
	active_spread  = LORA_SPREAD_12;
	/* datasheet ToA for 255 B at SF12/125k, worst case CR 4/8 with LDRO: about 14.03 s */
	TEST_ASSERT_EQ_UINT( "255 B at SF12/125k waits longer than its time on air", txdone_timeout_ms( 255 ) > 14030U, true );
	symbol_time_us = 1024U; /* SF8 @ 250 kHz */
	active_spread  = LORA_SPREAD_8;
	TEST_ASSERT_EQ_UINT( "Short packets get a short timeout", txdone_timeout_ms( 32 ) < 500U, true );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Zero-length payload is rejected" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "Zero length is rejected", lora_transmit_async( payload, 0 ), LORA_BUFFER_UNDERSIZED );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "DMA error releases the bus" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x80 | LORA_SLEEP_MODE;
	lora_transmit_async( payload, sizeof( payload ) );
	lora_process_async_error_cb();
	TEST_ASSERT_EQ_UINT( "Busy flag is released", dma_busy, false );
	TEST_ASSERT_EQ_UINT( "Not waiting for TxDone", dma_awaiting_txdone, false );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests the DMA receive path: RxDone, FIFO read over DMA, publish, CRC drop.
 */
void test_lora_receive_async
	(
	void
	)
{
uint8_t packet[3] = { 0xA1, 0xB2, 0xC3 };
uint8_t out[8];
uint8_t got = 0;

TEST_begin_nested_case( "RxDone reads the FIFO over DMA and publishes the packet" );
	{
	reset_test();
	dma_busy = false;
	dma_awaiting_txdone = false;
	atomic_store( &dma_ready, false );
	mock_spi_rx_data[0] = 0x00;             /* RegFifoRxCurrentAddr */
	mock_spi_rx_data[1] = sizeof( packet ); /* RegRxNbBytes */
	mock_spi_rx_data[2] = 0x40;             /* RegIrqFlags: RxDone */
	memcpy( &mock_spi_rx_data[3], packet, sizeof( packet ) );
	lora_process_dio0_cb();
	TEST_ASSERT_EQ_UINT( "Not ready until the DMA completes", lora_has_new_data(), false );
	lora_process_async_cb();
	TEST_ASSERT_EQ_UINT( "Packet is ready", lora_has_new_data(), true );
	TEST_ASSERT_EQ_UINT( "Bus is released", dma_busy, false );
	TEST_ASSERT_EQ_UINT( "Read succeeds", lora_get_latest( out, sizeof( out ), &got ), LORA_OK );
	TEST_ASSERT_EQ_MEMORY( "Payload matches the FIFO", out, packet, sizeof( packet ) );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "A payload CRC error is dropped without a DMA read" );
	{
	reset_test();
	dma_busy = false;
	dma_awaiting_txdone = false;
	atomic_store( &dma_ready, false );
	mock_spi_rx_data[0] = 0x00;
	mock_spi_rx_data[1] = sizeof( packet );
	mock_spi_rx_data[2] = 0x40 | LORA_IRQ_PAYLOAD_CRC_ERROR;
	TEST_ASSERT_EQ_UINT( "Bad CRC is reported", lora_request_receive_async(), LORA_RECEIVE_FAIL );
	TEST_ASSERT_EQ_UINT( "No DMA read is started", dma_pending_op, LORA_DMA_OP_NONE );
	TEST_ASSERT_EQ_UINT( "Bus is released", dma_busy, false );
	TEST_ASSERT_EQ_UINT( "Nothing is published", lora_has_new_data(), false );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests reading a completed DMA receive out of the double buffer.
 */
void test_lora_get_latest
	(
	void
	)
{
uint8_t packet[3] = { 0x11, 0x22, 0x33 };
uint8_t out[8];
uint8_t got;

TEST_begin_nested_case( "Nothing ready reports waiting" );
	{
	atomic_store( &dma_ready, false );
	TEST_ASSERT_EQ_UINT( "No data is waiting", lora_get_latest( out, sizeof( out ), &got ), LORA_WAITING );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "A ready packet is copied out and consumed" );
	{
	memcpy( dma_rx_buf[1], packet, sizeof( packet ) );
	dma_rx_bytes[1] = sizeof( packet );
	dma_ready_idx = 1;
	atomic_store( &dma_ready, true );
	got = 0;
	TEST_ASSERT_EQ_UINT( "Read succeeds", lora_get_latest( out, sizeof( out ), &got ), LORA_OK );
	TEST_ASSERT_EQ_MEMORY( "The published row is copied", out, packet, sizeof( packet ) );
	TEST_ASSERT_EQ_UINT( "Length is reported", got, sizeof( packet ) );
	TEST_ASSERT_EQ_UINT( "The packet is consumed", lora_has_new_data(), false );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "An undersized buffer leaves the packet ready" );
	{
	atomic_store( &dma_ready, true );
	got = 0;
	TEST_ASSERT_EQ_UINT( "Undersized buffer is rejected", lora_get_latest( out, 2, &got ), LORA_BUFFER_UNDERSIZED );
	TEST_ASSERT_EQ_UINT( "Length is not reported", got, 0 );
	TEST_ASSERT_EQ_UINT( "The packet is still ready", lora_has_new_data(), true );
	TEST_end_nested_case();
	}
}

/**
 * @brief Entry point for test
 * 
 * @return Exit status of test
 */
int main
	(
	void
	)
{
/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	/* lora.c */
	{ "lora_init_configure", test_lora_init_configure, "RQ.DRIVER.00009" },
	{ "lora_set_chip_mode", test_lora_set_chip_mode },
	{ "lora_reset", test_lora_reset },
	{ "lora_transmit_async", test_lora_transmit_async, "RQ.DRIVER.00007, RQ.DRIVER.00010" },
	{ "lora_receive_async", test_lora_receive_async, "RQ.DRIVER.00008" },
	{ "lora_get_latest", test_lora_get_latest, "RQ.DRIVER.00008" }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_set_type( TEST_TYPE_SW_INTEGRATION );
TEST_INITIALIZE_TEST( "lora", tests );

} /* main */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/