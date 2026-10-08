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

#include "lora_async.c"


/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
SPI_HandleTypeDef hspi4;  /* LoRa SPI */

extern uint8_t mock_spi_tx_buffer[MOCK_SPI_BUFFER_SIZE];
extern TELEMETRY_MESSAGE mock_telemetry_message;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/


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
	{ "async_tx", test_async_tx },
	{ "async_fsm_mode_select", test_async_fsm_mode_select }
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