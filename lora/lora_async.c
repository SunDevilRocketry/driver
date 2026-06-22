/*******************************************************************************
*
* FILE:
* 		lora_async.c
*
* DESCRIPTION:
* 		Contains API functions for transmating //{POSTPONED} and receiving// 
*       from the board's built-in LoRa module in non-blocking mode.
*
* COPYRIGHT:
*       Copyright (c) 2025 Sun Devil Rocketry.
*       All rights reserved.
*
*       This software is licensed under terms that can be found in the LICENSE
*       file in the root directory of this software component.
*       If no LICENSE file comes with this software, it is covered under the
*       BSD-3-Clause.
*
*       https://opensource.org/license/bsd-3-clause
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes
------------------------------------------------------------------------------*/
#include <string.h>

/*------------------------------------------------------------------------------
 MCU Pins
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
    #include "led.h"
#elif defined( GROUND_STATION    )
    #include "sdr_pin_defines_A0005.h"
    #include "led.h"
#elif defined( A0010             )
    #include "sdr_pin_defines_A0010.h"
    #include "led.h"
#endif

/*------------------------------------------------------------------------------
 Project Includes
------------------------------------------------------------------------------*/
#include "lora.h"
#include "telemetry.h"
#include "usb.h"
#include "main.h"

/*------------------------------------------------------------------------------
 Global Variables
------------------------------------------------------------------------------*/
static LORA_ASYNC_OP_MODE op_mode = LORA_ASYNC_TX;
static LORA_TX_FSM_STATE tx_fsm = LORA_TX_STATE_BLOCKING;

static LORA_STATUS lora_status = LORA_OK;

/* tx globals */
static uint8_t       register_contents[2] = {0x00, 0x00};
static TELEMETRY_MESSAGE  payload;
static uint8_t       burst_write_buf[TELEMETRY_MESSAGE_SIZE + 1];

/*------------------------------------------------------------------------------
 Static Prototypes
------------------------------------------------------------------------------*/

static void lora_tx_update
    (
    LORA_FSM_EVENT update_cause /* i: which kind of event triggered this update */
    );

/*------------------------------------------------------------------------------
 Procedures
------------------------------------------------------------------------------*/

void lora_fsm_update
    (
    LORA_FSM_EVENT update_cause
    )
{
switch ( op_mode )
    {
    case LORA_ASYNC_TX:
        lora_tx_update( update_cause );
        return;
    default: /* any unhandled mode -- just return */
        return;
    }

} /* lora_fsm_update */


LORA_STATUS lora_fsm_set_mode
    (
    LORA_ASYNC_OP_MODE new_mode
    )
{
/* Starting TX */
if( ( new_mode == LORA_ASYNC_TX )
 && ( op_mode  == LORA_ASYNC_OFF ) )
    {
    op_mode = LORA_ASYNC_TX;
    tx_fsm = LORA_TX_STATE_BLOCKING;
    }
/* Stopping TX */
else if( ( new_mode == LORA_ASYNC_OFF )
      && ( op_mode  == LORA_ASYNC_TX ) )
    {
    /* Signal to cancel */
    lora_tx_update( LORA_FSM_EVENT_CANCEL );
    }
else
    {
    lora_status = LORA_INVALID_CMD;
    }

return lora_status;

} /* lora_fsm_set_mode */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		lora_tx_update                                                           *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Update the lora async transmission FSM.                                  *
*                                                                                *
*********************************************************************************/
static void lora_tx_update
    (
    LORA_FSM_EVENT update_cause /* i: which kind of event triggered this update */
    )
{
/* Local Variables */

/* precondition: lora is not broken */
if( ( lora_status & ( LORA_FAIL | LORA_TRANSMIT_FAIL | LORA_TIMEOUT_FAIL ) )
 || ( update_cause == LORA_FSM_EVENT_CANCEL ) )
    {
    tx_fsm = LORA_TX_STATE_BLOCKING; /* cancel lora FSM */
    op_mode = LORA_ASYNC_OFF; /* stop handling lora events */
    return;
    }

// ETS TEMP: Test
// telemetry_get_next_message();
// lora_transmit( &payload, sizeof( TELEMETRY_MESSAGE ) );
// return;

/* update the current telemetry state */
switch( tx_fsm )
    {
    case LORA_TX_STATE_BLOCKING: /* FSM start */
        {
        /* Assumptions: LoRa is initialized with valid configs, telem should only initialize
           with a synchronous event from main loop. */
        if( update_cause != LORA_FSM_EVENT_SYNCHRONOUS_UPDATE )
            {
            /* do nothing */
            return;
            }
        tx_fsm = LORA_TX_STATE_STATUS_CHECK;
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case LORA_TX_STATE_STATUS_CHECK: /* check if in standby mode */
        {
        if( update_cause != LORA_FSM_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }

        /* check return. if not standby, cancel telem fsm and go back to blocking */
        if ((register_contents[1] & 0b111) != LORA_STANDBY_MODE)
            {
            tx_fsm = LORA_TX_STATE_BLOCKING; /* wait for next synchronous check */
            /* Preserve LoRa mode (bit 7) and other upper bits; only set mode. */
            lora_status = lora_write_register_IT(
                LORA_REG_OPERATION_MODE,
                (uint8_t)( ( register_contents[1] & (uint8_t) ~0x7 ) | (uint8_t) LORA_STANDBY_MODE )
                );
            return;
            }
        else /* success: go to next state*/
            {
            tx_fsm = LORA_TX_STATE_GETTING_BUF;
            lora_status = lora_read_register_IT(LORA_REG_FIFO_TX_BASE_ADDR, register_contents);
            }
        return;
        }
    
    case LORA_TX_STATE_GETTING_BUF: /* get fifo base ptr */
        {
        if( update_cause != LORA_FSM_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }

        tx_fsm = LORA_TX_STATE_SETTING_TX_BASE;
        lora_status = lora_write_register_IT(LORA_REG_FIFO_SPI_POINTER, register_contents[1]);
        return;
        }

    case LORA_TX_STATE_SETTING_TX_BASE: /* set tx base ptr */
        {
        if( update_cause != LORA_FSM_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }

        tx_fsm = LORA_TX_STATE_WRITING_MSG_LEN;
        lora_status = lora_write_register_IT(LORA_REG_SIGNAL_TO_NOISE, TELEMETRY_MESSAGE_SIZE);
        return;
        }

    case LORA_TX_STATE_WRITING_MSG_LEN: /* write the length of the lora message */
        {
        if( update_cause != LORA_FSM_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }

        tx_fsm = LORA_TX_STATE_WRITING_MSG;
        telemetry_get_next_message(&payload);
        burst_write_buf[0] = (LORA_REG_FIFO_RW | 0x80); /* set up reg write */
        memcpy(&(burst_write_buf[1]), &payload, TELEMETRY_MESSAGE_SIZE);
        lora_status = lora_write_IT(burst_write_buf, TELEMETRY_MESSAGE_SIZE + 1);
        return;
        }
    
    case LORA_TX_STATE_WRITING_MSG: /* writing the message itself */
        {
        if( update_cause != LORA_FSM_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* check status register */
        tx_fsm = LORA_TX_STATE_PRE_TX_STATUS_CHECK;
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case LORA_TX_STATE_PRE_TX_STATUS_CHECK: /* writing the message itself */
        {
        if( update_cause != LORA_FSM_EVENT_REG_READ_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* switch to TX mode */
        tx_fsm = LORA_TX_STATE_STARTING_TRANSMISSION;
        uint8_t new_opmode_register = (register_contents[1] & ~(0x7));
        new_opmode_register = (new_opmode_register | LORA_TRANSMIT_MODE);
        lora_status = lora_write_register_IT( LORA_REG_OPERATION_MODE, new_opmode_register );
        return;
        }
    
    case LORA_TX_STATE_STARTING_TRANSMISSION:
        {
        if( update_cause != LORA_FSM_EVENT_WRITE_CPLT )
            {
            /* do nothing */
            return;
            }
        
        /* opmode change complete, we are now transmitting */
        tx_fsm = LORA_TX_STATE_TRANSMITTING;
        register_contents[1] = 0xFF; /* set this to FF so we can detect when the contents have changed */
        lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
        return;
        }

    case LORA_TX_STATE_TRANSMITTING:
        {
        if( ( update_cause != LORA_FSM_EVENT_EXTI_RAISED )
         && ( update_cause != LORA_FSM_EVENT_REG_READ_CPLT ) )
            {
            /* do nothing */
            return;
            }
        
        if( (register_contents[1] & 0b111) == LORA_STANDBY_MODE ) 
            {
            /* transmission is complete! start the buffer retrieval operation and jump higher on the FSM */
            tx_fsm = LORA_TX_STATE_GETTING_BUF;
            lora_status = lora_read_register_IT(LORA_REG_FIFO_TX_BASE_ADDR, register_contents);
            }
        else
            {
            lora_status = lora_read_register_IT(LORA_REG_OPERATION_MODE, register_contents);
            }
        return;
        }
        
    }


} /* lora_tx_update */