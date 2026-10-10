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

// Allow mutation of static memory by including source files directly
#include "lora.c"
#include "lora_async.c"


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
extern TELEMETRY_MESSAGE mock_telemetry_message;
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
memset( &mock_telemetry_message, 0, sizeof( mock_telemetry_message ) );
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
 * @brief Test async transmission FSM
 */
void test_async_tx
	(
	void
	)
{
struct TEST_CASE {
	const char* description;
	LORA_TX_FSM_STATE starting_state;
	LORA_FSM_EVENT update_cause;
	uint8_t last_read_contents;
	bool is_write;
	uint8_t expected_rw_address;
	uint8_t expected_first_write_byte;
	LORA_TX_FSM_STATE expected_state;
} TEST_CASE;

struct TEST_CASE test_cases[] = {
	{ "Blocking -> Status Check (Synchro)", LORA_TX_STATE_BLOCKING, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0x00, false, LORA_REG_OPERATION_MODE, 0x00, LORA_TX_STATE_STATUS_CHECK },
	{ "Blocking -> Blocking (REG_RD)", LORA_TX_STATE_BLOCKING, LORA_FSM_EVENT_REG_READ_CPLT, 0x00, false, LORA_REG_OPERATION_MODE, 0x00, LORA_TX_STATE_BLOCKING },
	{ "Status Check -> Getting Buf (REG_RD)", LORA_TX_STATE_STATUS_CHECK, LORA_FSM_EVENT_REG_READ_CPLT, 0x01, false, LORA_REG_FIFO_TX_BASE_ADDR, 0x00, LORA_TX_STATE_GETTING_BUF },
	{ "Status Check -> Blocking (REG_RD)", LORA_TX_STATE_STATUS_CHECK, LORA_FSM_EVENT_REG_READ_CPLT, 0x03, true, LORA_REG_OPERATION_MODE, 0x01, LORA_TX_STATE_BLOCKING },
	{ "Status Check -> Status Check (Synchro)", LORA_TX_STATE_STATUS_CHECK, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0x00, false, LORA_REG_OPERATION_MODE, 0x00, LORA_TX_STATE_STATUS_CHECK },
	{ "Getting Buf -> Set TX Base (REG_RD)", LORA_TX_STATE_GETTING_BUF, LORA_FSM_EVENT_REG_READ_CPLT, 0xBB, true, LORA_REG_FIFO_SPI_POINTER, 0xBB, LORA_TX_STATE_SETTING_TX_BASE },
	{ "Getting Buf -> Getting Buf (Synchro)", LORA_TX_STATE_GETTING_BUF, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_FIFO_SPI_POINTER, 0xBB, LORA_TX_STATE_GETTING_BUF },
	{ "Set TX Base -> Msg Len (REG_WRT)", LORA_TX_STATE_SETTING_TX_BASE, LORA_FSM_EVENT_WRITE_CPLT, 0xBB, true, LORA_REG_SIGNAL_TO_NOISE, TELEMETRY_MESSAGE_SIZE, LORA_TX_STATE_WRITING_MSG_LEN },
	{ "Set TX Base -> Set TX Base (Synchro)", LORA_TX_STATE_SETTING_TX_BASE, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_SIGNAL_TO_NOISE, 0xBB, LORA_TX_STATE_SETTING_TX_BASE },
	{ "Msg Len -> Write Msg (REG_WRT)", LORA_TX_STATE_WRITING_MSG_LEN, LORA_FSM_EVENT_WRITE_CPLT, 0xCC, true, LORA_REG_FIFO_RW, 0xCC, LORA_TX_STATE_WRITING_MSG },
	{ "Msg Len -> Msg Len (Synchro)", LORA_TX_STATE_WRITING_MSG_LEN, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_FIFO_RW, 0xBB, LORA_TX_STATE_WRITING_MSG_LEN },
	{ "Write Msg -> Pre-TX (REG_WRT)", LORA_TX_STATE_WRITING_MSG, LORA_FSM_EVENT_WRITE_CPLT, 0xBB, false, LORA_REG_OPERATION_MODE, 0xBB, LORA_TX_STATE_PRE_TX_STATUS_CHECK },
	{ "Write Msg -> Write Msg (Synchro)", LORA_TX_STATE_WRITING_MSG, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_FIFO_RW, 0xBB, LORA_TX_STATE_WRITING_MSG },
	{ "Pre-TX -> Start TX (REG_RD)", LORA_TX_STATE_PRE_TX_STATUS_CHECK, LORA_FSM_EVENT_REG_READ_CPLT, 0x81, true, LORA_REG_OPERATION_MODE, 0x83, LORA_TX_STATE_STARTING_TRANSMISSION },
	{ "Pre-TX -> Start TX (Synchro)", LORA_TX_STATE_PRE_TX_STATUS_CHECK, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_OPERATION_MODE, 0xBB, LORA_TX_STATE_PRE_TX_STATUS_CHECK },
	{ "Start TX -> TX (REG_WRT)", LORA_TX_STATE_STARTING_TRANSMISSION, LORA_FSM_EVENT_WRITE_CPLT, 0x81, false, LORA_REG_OPERATION_MODE, 0x83, LORA_TX_STATE_TRANSMITTING },
	{ "Start TX -> Start TX (Synchro)", LORA_TX_STATE_STARTING_TRANSMISSION, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0xBB, true, LORA_REG_OPERATION_MODE, 0xBB, LORA_TX_STATE_STARTING_TRANSMISSION },
	{ "TX -> Get Buf (REG_RD)", LORA_TX_STATE_TRANSMITTING, LORA_FSM_EVENT_REG_READ_CPLT, 0x81, false, LORA_REG_FIFO_TX_BASE_ADDR, 0xBB, LORA_TX_STATE_GETTING_BUF },
	{ "TX -> TX (REG_RD)", LORA_TX_STATE_TRANSMITTING, LORA_FSM_EVENT_REG_READ_CPLT, 0x83, false, LORA_REG_OPERATION_MODE, 0xBB, LORA_TX_STATE_TRANSMITTING },
	{ "TX -> TX (Synchro)", LORA_TX_STATE_TRANSMITTING, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0x83, false, LORA_REG_OPERATION_MODE, 0xBB, LORA_TX_STATE_TRANSMITTING },
	{ "Robustness: Invalid -> Invalid", 0xCC, LORA_FSM_EVENT_SYNCHRONOUS_UPDATE, 0x83, false, LORA_REG_OPERATION_MODE, 0xBB, 0xCC },
	};

for( int i = 0; i < array_size( test_cases ); i++ )
	{
	TEST_begin_nested_case( test_cases[i].description );

	/* Set up parameters */
	memset( &mock_telemetry_message, 0xCC, TELEMETRY_MESSAGE_SIZE ); // reserved sentinel value for the message write
	reset_test();
	op_mode = LORA_ASYNC_TX;
	tx_fsm = test_cases[i].starting_state;
	register_contents[1] = test_cases[i].last_read_contents;

	/* Call FUT */
	lora_fsm_update( test_cases[i].update_cause );

	/* Verify results */
	if( test_cases[i].starting_state == test_cases[i].expected_state )
		{
		TEST_ASSERT_EQ_UINT( "Verify that the state was not updated", tx_fsm, test_cases[i].expected_state );
		}
	else
		{
		TEST_ASSERT_EQ_UINT( "Verify that the state was updated properly", tx_fsm, test_cases[i].expected_state );
		
		if( test_cases[i].is_write )
			{
			TEST_ASSERT_EQ_UINT( "Verify that the correct address was written", mock_spi_tx_buffer[0], test_cases[i].expected_rw_address | 0x80 );
			if( test_cases[i].starting_state == LORA_TX_STATE_WRITING_MSG_LEN
			 && test_cases[i].expected_state == LORA_TX_STATE_WRITING_MSG )
				{
				TEST_ASSERT_EQ_MEMORY( "Verify that the telemetry message was written", &(mock_spi_tx_buffer[1]), &mock_telemetry_message, TELEMETRY_MESSAGE_SIZE );
				}
			else
				{
				TEST_ASSERT_EQ_UINT( "Verify that the correct contents were written", mock_spi_tx_buffer[1], test_cases[i].expected_first_write_byte );		
				}
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Verify that the correct address was read", mock_spi_tx_buffer[0], test_cases[i].expected_rw_address );
			}
		}

	TEST_end_nested_case();
	}

/* Additional robustness case for top level function */
TEST_begin_nested_case( "Robustness: State doesn't change if the op mode is invalid" );
	{
	/* Set up parameters */
	reset_test();
	op_mode = 0xCC;
	tx_fsm = LORA_TX_STATE_WRITING_MSG;
	
	/* Call FUT */
	lora_fsm_update( LORA_FSM_EVENT_WRITE_CPLT );

	/* Verify that the opmode and fsm state have not changed */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was not changed.", op_mode, 0xCC );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was not changed.", tx_fsm, LORA_TX_STATE_WRITING_MSG );
	TEST_end_nested_case();
	}

} /* test_async_tx */


/**
 * @brief Tests asynchronous mode select
 */
void test_async_fsm_mode_select
	(
	void
	)
{
TEST_begin_nested_case( "Case 1: Verify that the TX FSM is stopped if moved to the cancel mode." );
	{
	/* Set up test */
	reset_test();
	op_mode = LORA_ASYNC_TX;
	tx_fsm = LORA_TX_STATE_WRITING_MSG_LEN;

	/* Call FUT */
	lora_fsm_set_mode( LORA_ASYNC_OFF );

	/* Verify result */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was set to ASYNC_OFF", op_mode, LORA_ASYNC_OFF );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was set to BLOCKING", tx_fsm, LORA_TX_STATE_BLOCKING );

	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Case 2: Verify that the TX FSM is started if moved to the start mode." );
	{
	/* Set up test */
	reset_test();
	op_mode = LORA_ASYNC_OFF;
	tx_fsm = 0xFF;

	/* Call FUT */
	lora_fsm_set_mode( LORA_ASYNC_TX );

	/* Verify result */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was set to ASYNC_OFF", op_mode, LORA_ASYNC_TX );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was set to BLOCKING (fsm started)", tx_fsm, LORA_TX_STATE_BLOCKING );

	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Case 3: Verify that the function returns LORA_INVALID_CMD if the operation is unsupported (TX -> TX)." );
	{
	/* Set up test */
	reset_test();
	op_mode = LORA_ASYNC_TX;
	tx_fsm = 0xFF;

	/* Call FUT */
	TEST_ASSERT_EQ_UINT( "Verify that the function returned invalid", lora_fsm_set_mode( LORA_ASYNC_TX ), LORA_INVALID_CMD );

	/* Verify result */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was not changed.", op_mode, LORA_ASYNC_TX );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was not changed.", tx_fsm, 0xFF );

	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Case 4: Verify that the function returns LORA_INVALID_CMD if the operation is unsupported (OFF -> OFF)." );
	{
	/* Set up test */
	reset_test();
	op_mode = LORA_ASYNC_OFF;
	tx_fsm = 0xFF;

	/* Call FUT */
	TEST_ASSERT_EQ_UINT( "Verify that the function returned invalid", lora_fsm_set_mode( LORA_ASYNC_OFF ), LORA_INVALID_CMD );

	/* Verify result */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was not changed.", op_mode, LORA_ASYNC_OFF );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was not changed.", tx_fsm, 0xFF );

	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Case 5: Verify that the TX FSM is stopped if lora status has a fail." );
	{
	/* Set up test */
	reset_test();
	op_mode = LORA_ASYNC_TX;
	tx_fsm = LORA_TX_STATE_WRITING_MSG_LEN;
	lora_status = LORA_TRANSMIT_FAIL;

	/* Call FUT */
	lora_fsm_update( LORA_FSM_EVENT_WRITE_CPLT );

	/* Verify result */
	TEST_ASSERT_EQ_UINT( "Verify that the mode was set to ASYNC_OFF", op_mode, LORA_ASYNC_OFF );
	TEST_ASSERT_EQ_UINT( "Verify that the FSM was set to BLOCKING", tx_fsm, LORA_TX_STATE_BLOCKING );

	TEST_end_nested_case();
	}
}

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
 * @brief Tests blocking LoRa transmission.
 */
void test_lora_transmit
	(
	void
	)
{
uint8_t rx_values[MOCK_SPI_BUFFER_SIZE] = { 0 };
uint8_t data[3] = { 0xA1, 0xB2, 0xC3 };

TEST_begin_nested_case( "Transmit sends payload and returns success" );
	{
	reset_test();
	rx_values[0] = 0x80; /* Existing operation mode */
	rx_values[1] = 0x33; /* TX FIFO base */
	rx_values[2] = 0x81; /* Existing operation mode */
	rx_values[3] = 0x81; /* TX complete: standby */
	memcpy( mock_spi_rx_data, rx_values, sizeof( rx_values ) );
	mock_hal_tick_step = 10;
	TEST_ASSERT_EQ_UINT( "Packet transmission succeeds", lora_transmit( data, sizeof( data ) ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Transmit history includes the payload bytes", mock_spi_history_contains( data, sizeof( data ) ), true );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Transmit returns failure when SPI setup fails" );
	{
	reset_test();
	MOCK_HAL_Status_Return( HAL_ERROR );
	TEST_ASSERT_EQ_UINT( "SPI failure aborts transmission", lora_transmit( data, sizeof( data ) ), LORA_FAIL );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Transmit returns failure when SPI transmit fails" );
	{
	reset_test();
	mock_spi_fail_tx_call = 5; /* Fail the payload-length register write */
	MOCK_HAL_Status_Return( HAL_OK );
	TEST_ASSERT_EQ_UINT( "SPI failure aborts transmission", lora_transmit( data, sizeof( data ) ), LORA_FAIL );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Transmit times out during normal tick progression" );
	{
	reset_test();
	rx_values[0] = 0x80; /* Existing operation mode */
	rx_values[1] = 0x33; /* TX FIFO base */
	rx_values[2] = 0x81; /* Existing operation mode */
	rx_values[3] = 0x83; /* Still transmitting */
	rx_values[4] = 0x83; /* Still transmitting */
	memcpy( mock_spi_rx_data, rx_values, sizeof( rx_values ) );
	mock_hal_tick = 0;
	mock_hal_tick_step = 50;
	TEST_ASSERT_EQ_UINT( "Transmission fails after its timeout", lora_transmit( data, sizeof( data ) ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Polling continues until the timeout", mock_spi_rx_call_count, 5 );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests LoRa packet readiness and receive handling.
 */
void test_lora_receive
	(
	void
	)
{
uint8_t data[3] = { 0xA1, 0xB2, 0xC3 };
uint8_t received[4] = { 0 };
uint8_t received_count = 0;

TEST_begin_nested_case( "Receive-ready reports packets only in continuous RX mode" );
	{
	reset_test();
	mock_spi_rx_data[0] = LORA_RX_CONTINUOUS_MODE;
	mock_spi_rx_data[1] = 0x40;
	TEST_ASSERT_EQ_UINT( "RX-done IRQ is reported as ready", lora_receive_ready(), LORA_READY );

	reset_test();
	mock_spi_rx_data[0] = LORA_STANDBY_MODE;
	TEST_ASSERT_EQ_UINT( "Non-RX mode is rejected", lora_receive_ready(), LORA_FAIL );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Receive-ready reports waiting when no RX-done IRQ is set" );
	{
	reset_test();
	mock_spi_rx_data[0] = LORA_RX_CONTINUOUS_MODE;
	mock_spi_rx_data[1] = 0x00;
	TEST_ASSERT_EQ_UINT( "No RX-done IRQ reports waiting", lora_receive_ready(), LORA_WAITING );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Receive returns waiting until RX data is ready" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "Receive reports waiting", lora_receive( received, sizeof( received ), &received_count ), LORA_WAITING );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Receive returns undersized when the caller buffer is too small" );
	{
	reset_test();
	mock_spi_rx_data[0] = LORA_RX_CONTINUOUS_MODE;
	mock_spi_rx_data[1] = 0x40;
	(void)lora_receive_ready();
	mock_spi_rx_offset = 0;
	mock_spi_rx_data[0] = 0x00; /* IRQ flags */
	mock_spi_rx_data[1] = 0x05; /* Five bytes received */
	TEST_ASSERT_EQ_UINT( "Small buffer is rejected", lora_receive( received, sizeof( received ), &received_count ), LORA_BUFFER_UNDERSIZED );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Receive preserves the driver's current CRC-error behavior" );
	{
	reset_test();
	mock_spi_rx_data[0] = LORA_RX_CONTINUOUS_MODE;
	mock_spi_rx_data[1] = 0x40;
	(void)lora_receive_ready();
	mock_spi_rx_offset = 0;
	mock_spi_rx_data[0] = 0x20; /* IRQ flags with CRC error */
	received_count = 0xEE;
	TEST_ASSERT_EQ_UINT( "CRC error currently returns success", lora_receive( received, sizeof( received ), &received_count ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "CRC error does not update the received count", received_count, 0xEE );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Receive reads packet bytes and returns their count" );
	{
	reset_test();
	mock_spi_rx_data[0] = LORA_RX_CONTINUOUS_MODE;
	mock_spi_rx_data[1] = 0x40;
	(void)lora_receive_ready();
	mock_spi_rx_offset = 0;
	mock_spi_rx_data[0] = 0x00; /* IRQ flags */
	mock_spi_rx_data[1] = 3;    /* Number of bytes */
	mock_spi_rx_data[2] = 0x22; /* RX FIFO base */
	mock_spi_rx_data[3] = 0xA1;
	mock_spi_rx_data[4] = 0xB2;
	mock_spi_rx_data[5] = 0xC3;
	TEST_ASSERT_EQ_UINT( "Packet receive succeeds", lora_receive( received, sizeof( received ), &received_count ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Received byte count is returned", received_count, 3 );
	TEST_ASSERT_EQ_MEMORY( "Packet data is copied into caller buffer", received, data, sizeof( data ) );
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
 * @brief Tests interrupt-driven LoRa register reads.
 */
void test_lora_read_register_it
	(
	void
	)
{
uint8_t it_read_buffer[2] = { 0 };

TEST_begin_nested_case( "Interrupt-driven register read starts successfully and masks the address" );
	{
	reset_test();
	mock_spi_rx_data[1] = 0x44;
	TEST_ASSERT_EQ_UINT( "IT register read starts", _lora_read_register_IT( 0xC2, it_read_buffer ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Read address has its write bit cleared", mock_spi_tx_buffer[0], 0x42 );
	TEST_ASSERT_EQ_UINT( "Read transfer is two bytes", mock_spi_tx_size, 2 );
	TEST_ASSERT_EQ_UINT( "Second read byte contains the register response", it_read_buffer[1], 0x44 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Interrupt-driven register read does not start successfully." );
	{
	reset_test();
	MOCK_HAL_Status_Return( 1 );
	mock_spi_rx_data[1] = 0x44;
	TEST_ASSERT_EQ_UINT( "IT register read reports the error", _lora_read_register_IT( 0xC2, it_read_buffer ), LORA_FAIL );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests interrupt-driven LoRa register writes.
 */
void test_lora_write_register_it
	(
	void
	)
{
TEST_begin_nested_case( "Interrupt-driven register write captures address and value" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "IT register write starts", _lora_write_register_IT( 0x42, 0x12 ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Write address has its write bit set", mock_spi_tx_buffer[0], 0xC2 );
	TEST_ASSERT_EQ_UINT( "Write value is captured", mock_spi_tx_buffer[1], 0x12 );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests interrupt-driven raw LoRa writes.
 */
void test_lora_write_it
	(
	void
	)
{
uint8_t data[3] = { 0xA1, 0xB2, 0xC3 };

TEST_begin_nested_case( "Interrupt-driven buffer write preserves the supplied bytes" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "IT buffer write starts", _lora_write_IT( data, sizeof( data ) ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Buffer length is captured", mock_spi_tx_size, sizeof( data ) );
	TEST_ASSERT_EQ_MEMORY( "Buffer contents are captured", mock_spi_tx_buffer, data, sizeof( data ) );
	MOCK_HAL_Status_Return( HAL_ERROR );
	TEST_ASSERT_EQ_UINT( "IT write reports HAL errors", _lora_write_IT( data, sizeof( data ) ), LORA_FAIL );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests blocking reads from a LoRa register buffer.
 */
void test_lora_read_register_buffer
	(
	void
	)
{
uint8_t data[3] = { 0 };

TEST_begin_nested_case( "Register-buffer read captures data and masks the address" );
	{
	reset_test();
	mock_spi_rx_data[0] = 0x11;
	mock_spi_rx_data[1] = 0x22;
	mock_spi_rx_data[2] = 0x33;
	TEST_ASSERT_EQ_UINT( "Register-buffer read succeeds", read_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Read address is transmitted", mock_spi_tx_buffer[0], LORA_REG_FIFO_RW );
	TEST_ASSERT_EQ_MEMORY( "Read bytes are copied to the destination", data, mock_spi_rx_data, sizeof( data ) );
	TEST_ASSERT_EQ_UINT( "Chip select is asserted and released", mock_gpio_write_count, 2 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Register-buffer read reports transmit failure" );
	{
	reset_test();
	mock_spi_fail_tx_call = 1;
	TEST_ASSERT_EQ_UINT( "Transmit failure is reported", read_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Chip select is released after transmit failure", mock_gpio_write_count, 2 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Register-buffer read reports receive failure" );
	{
	reset_test();
	mock_spi_fail_rx_call = 1;
	TEST_ASSERT_EQ_UINT( "Receive failure is reported", read_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Chip select is released after receive failure", mock_gpio_write_count, 2 );
	TEST_end_nested_case();
	}
}

/**
 * @brief Tests blocking writes to a LoRa register buffer.
 */
void test_lora_write_register_buffer
	(
	void
	)
{
uint8_t data[3] = { 0xA1, 0xB2, 0xC3 };
uint8_t expected[sizeof( data ) + 1] = { LORA_REG_FIFO_RW | 0x80, 0xA1, 0xB2, 0xC3 };

TEST_begin_nested_case( "Register-buffer write sends address and payload" );
	{
	reset_test();
	TEST_ASSERT_EQ_UINT( "Register-buffer write succeeds", write_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_OK );
	TEST_ASSERT_EQ_UINT( "Register address and payload are in history", mock_spi_history_contains( expected, sizeof( expected ) ), true );
	TEST_ASSERT_EQ_UINT( "Both transfers are sent", mock_spi_tx_call_count, 2 );
	TEST_ASSERT_EQ_UINT( "Chip select is asserted and released", mock_gpio_write_count, 2 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Register-buffer write reports first transmit failure" );
	{
	reset_test();
	mock_spi_fail_tx_call = 1;
	TEST_ASSERT_EQ_UINT( "First transmit failure is reported", write_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Chip select is released after failure", mock_gpio_write_count, 2 );
	TEST_end_nested_case();
	}

TEST_begin_nested_case( "Register-buffer write reports payload transmit failure" );
	{
	reset_test();
	mock_spi_fail_tx_call = 2;
	TEST_ASSERT_EQ_UINT( "Payload transmit failure is reported", write_register_buffer( LORA_REG_FIFO_RW, data, sizeof( data ) ), LORA_FAIL );
	TEST_ASSERT_EQ_UINT( "Chip select is released after failure", mock_gpio_write_count, 2 );
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
	/* lora_async.c */
	{ "async_tx", test_async_tx, "RQ.DRIVER.00007, RQ.DRIVER.00010" },
	{ "async_fsm_mode_select", test_async_fsm_mode_select, "RQ.DRIVER.00010" },
	/* lora.c */
	{ "lora_init_configure", test_lora_init_configure, "RQ.DRIVER.00009" },
	{ "lora_transmit", test_lora_transmit, "RQ.DRIVER.00007" },
	{ "lora_receive", test_lora_receive, "RQ.DRIVER.00008" },
	{ "lora_set_chip_mode", test_lora_set_chip_mode },
	{ "lora_reset", test_lora_reset },
	{ "lora_read_register_it", test_lora_read_register_it },
	{ "lora_write_register_it", test_lora_write_register_it },
	{ "lora_write_it", test_lora_write_it },
	{ "lora_read_register_buffer", test_lora_read_register_buffer },
	{ "lora_write_register_buffer", test_lora_write_register_buffer }
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