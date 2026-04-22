/*******************************************************************************
*
* FILE: 
* 		gps.c
*
* DESCRIPTION: 
* 		Contains API functions for GPS 
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
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "timer.h"
#include "gps.h"
#include "led.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 Preprocesor Directives 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/
static uint32_t gps_init_tick = 0;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       gps_check_ack                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Read and validate UBX ACK-ACK/ACK-NAK response                        *
*       Returns GPS_OK on ACK-ACK, GPS_ERROR on ACK-NAK or timeout            *
*                                                                              *
*******************************************************************************/
static GPS_STATUS gps_check_ack
    (
    void
    )
{
/* local vars */
uint8_t    ack_buf[10];
GPS_STATUS gps_status;
uint8_t  msg_class = 0x06;   /* expected class byte of the sent message */
uint8_t  msg_id = 0x8A;      /* expected ID byte of the sent message */

/* Read 10-byte ACK frame */
gps_status = gps_receive( (void*)ack_buf, sizeof(ack_buf), GPS_DEFAULT_TIMEOUT * 10 );

/* Validate preamble */
if( ack_buf[0] != 0xB5 || ack_buf[1] != 0x62 )
    {
    return GPS_FAIL;
    }

/* Validate class is ACK (0x05) */
if( ack_buf[2] != 0x05 )
    {
    return GPS_FAIL;
    }

/* Check ACK-ACK (0x01) vs ACK-NAK (0x00) */
if( ack_buf[3] != 0x01 )
    {
    return GPS_FAIL;
    }

/* Validate the ACK refers to our message */
if( ack_buf[6] != msg_class || ack_buf[7] != msg_id )
    {
    return GPS_FAIL;
    }

return gps_status;

} /* gps_check_ack */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		gps_init                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Set up configuration messages for GPS.                                 *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_init
    (
    void
    ) 
{
/* local vars */
GPS_STATUS gps_status = GPS_OK;

/* magic byte sequences for init*/
#include "gps_init.inc"

/* Trigger reset */
HAL_GPIO_WritePin( GPS_RST_PORT, GPS_RST_PIN, GPIO_PIN_RESET );
delay_ms( 10 );
HAL_GPIO_WritePin( GPS_RST_PORT, GPS_RST_PIN, GPIO_PIN_SET );

delay_ms( GPS_STARTUP_DELAY ); /* busy-wait until recommended startup delay */

gps_init_tick = HAL_GetTick();

// static const uint8_t nmea_gns_poll[] = "$EIGNQ,GNS*21\r\n";
// volatile static uint8_t recv[80];
// gps_status |= gps_transmit
//     (
//     (void*)nmea_gns_poll,
//     sizeof( nmea_gns_poll ) - 1,
//     GPS_DEFAULT_TIMEOUT
//     );
// gps_status |= gps_receive(recv, 80, GPS_DEFAULT_TIMEOUT);

// /* Enter UBX config mode */
// gps_status |= gps_transmit
//     (
//     (void*)nmea_enable_ubx_input,
//     sizeof( nmea_enable_ubx_input ) - 1,
//     GPS_DEFAULT_TIMEOUT
//     );

/* Set up UART baud */
// gps_status |= gps_transmit
//     (
//     (void*)uart_baud_config, 
//     sizeof( uart_baud_config ), 
//     HAL_DEFAULT_TIMEOUT * 10
//     ); /* must set GPS baud while on the default rate */
// gps_status |= gps_check_ack();

// gps_status |= HAL_UART_DeInit(&GPS_HUART); /* de-init the peripheral before reconfiguring */
// GPS_HUART.Init.BaudRate = 921600;
// gps_status |= HAL_UART_Init(&GPS_HUART); /* re-init the peripheral with the updated baud */

return gps_status;

} /* gps_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		gps_start                                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Start receiving GPS messages.                                          *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_start
    (
    uint8_t* gps_mesg_ptr /* pointer to global message byte */
    )
{
/* Trigger GPS */
GPS_STATUS gps_status = GPS_OK;
gps_status = gps_receive_IT( gps_mesg_ptr, 1 );

/* Determine status */
if( gps_status != GPS_OK )
    {
    return gps_status;
    }
else if( gps_init_tick + GPS_RECEIVE_DELAY > HAL_GetTick() )
    {
    return GPS_BUSY;
    }
else
    {
    return GPS_OK;
    }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		gps_transmit                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over USB                         *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Transmit( &( GPS_HUART ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( gps_status != HAL_OK )
	{
	return gps_status;
	}
else
	{
	return GPS_OK;
	}

} /* usb_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_recieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Receive( &( GPS_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );
/* Return HAL status */
switch ( gps_status )
	{
	case HAL_TIMEOUT:
		{
		return GPS_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return GPS_OK;
		break;
		}
	default:
		{
		return GPS_FAIL;
		break;
        }
	}

} /* usb_receive */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_recieve_IT                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
GPS_STATUS gps_receive_IT 
	(
	uint8_t*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size /* Size of the data to be received */
	)
{
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef gps_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
gps_status = HAL_UART_Receive_IT( &( GPS_HUART ),
                               rx_data_ptr   , 
                               rx_data_size );

/* Return HAL status */
switch ( gps_status )
	{
	case HAL_TIMEOUT:
		{
		return GPS_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return GPS_OK;
		break;
		}
	default:
		{
		return GPS_FAIL;
		break;
        }
	}

} /* usb_receive_IT */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_mesg_validate                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Validate message returned from GPS                                     *
*                                                                              *
* TEST:                                                                        *
*       test_gps_mesg_validate provides coverage. If this function             *
*       is updated, make sure the test cases are updated to match.             *
*                                                                              *
*******************************************************************************/
int gps_mesg_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
} /*gps_mesg_validate*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		GPS_parse                                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Convert raw NMEA string to usable data                                 *
*                                                                              *
* TEST:                                                                        *
*       test_GPS_parse provides coverage. If this function or its              *
*       helpers are updated, make sure the test cases are updated to match.    *
*                                                                              *
*******************************************************************************/
void GPS_parse(GPS_DATA* gps_ptr, char *GPSstrParse){
/* Get message type */
char token[8]; // Needs to be 8 chars for memory alignment
strncpy(token, GPSstrParse, 6);
token[6] = '\0';
int idx = 7; /* Skips "$GPXXX,"*/

/* Parse by message type */
if (!strcmp(token, "$GPGGA") || !strcmp(token, "$GNGGA")) 
    {
    gps_ptr->utc_time = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ns = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ew = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->lock = (int)gps_string_to_float(GPSstrParse, &idx) + 0.5;
    gps_ptr->satelites = (int)(gps_string_to_float(GPSstrParse, &idx) + 0.5); // This is a decimal number.
    gps_ptr->hdop = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->msl_altitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->msl_units = gps_string_to_char(GPSstrParse, &idx);
    gps_conv_latitude_longitude( gps_ptr );
    }
else if (!strcmp(token, "$GPRMC") || !strcmp(token, "$GNRMC")) 
    {
    gps_ptr->utc_time = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->rmc_status = gps_string_to_char(GPSstrParse, &idx); /* unused */
    gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ns = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ew = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->speed_k = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->course_d = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->date = (int)(0.5 + gps_string_to_float(GPSstrParse, &idx));
    gps_conv_latitude_longitude( gps_ptr );
    }
else if (!strcmp(token, "$GPGLL") || !strcmp(token, "$GNGLL")) 
    {
    gps_ptr->nmea_latitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ns = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->nmea_longitude = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->ew = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->utc_time = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->gll_status = gps_string_to_char(GPSstrParse, &idx);
    gps_conv_latitude_longitude( gps_ptr );
    }
else if (!strcmp(token, "$GPVTG") || !strcmp(token, "$GNVTG")) 
    {
    gps_ptr->course_t = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->course_t_unit = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->course_m = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->course_m_unit = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->speed_k = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->speed_k_unit = gps_string_to_char(GPSstrParse, &idx);
    gps_ptr->speed_km = gps_string_to_float(GPSstrParse, &idx);
    gps_ptr->speed_km_unit = gps_string_to_char(GPSstrParse, &idx);
    }
else if (!strcmp(token, "$GPTXT") || !strcmp(token, "$GNTXT"))
    {
    led_set_color( LED_WHITE );
    }
} /* GPS_parse */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_string_to_float                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Convert part of an NMEA string to a float                              *
*                                                                              *
* TEST:                                                                        *
*       test_GPS_parse provides coverage. If this function is                  *
*       updated, please re-run the test and update if necessary                *
*                                                                              *
*******************************************************************************/
float gps_string_to_float(char *GPSstrParse, int* inputIdx) 
{
int idx = *inputIdx;
char currChar = GPSstrParse[idx];
char tempstr[16];
int tempidx = 0;
if (GPSstrParse[idx] == ',') /* Checks if subsequent comma */
    {
    *inputIdx = *inputIdx + 1;
    return 0.0f; /* null return */
    }
while (currChar != ',') 
    {
    if (tempidx > 15) 
        {
        /* ERROR HANDLING */
        // maybe just exit loop? and deal with bad data? or make it null.
        return 0.0f;
        }
    tempstr[tempidx] = GPSstrParse[idx];
    tempidx++;
    idx++;
    currChar = GPSstrParse[idx];
    }
*inputIdx = idx + 1;
tempstr[tempidx] = '\0';
return strtof(tempstr, NULL);
} /* gps_string_to_float */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_string_to_char                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Get a char from an NMEA string if it exists                            *
*                                                                              *
* TEST:                                                                        *
*       test_GPS_parse provides coverage. If this function is                  *
*       updated, please re-run the test and update if necessary                *
*                                                                              *
*******************************************************************************/
char gps_string_to_char(char *GPSstrParse, int* inputIdx) 
{
int idx = *inputIdx;
if (GPSstrParse[idx] == ',') /* Checks if subsequent comma */
    {
    *inputIdx = *inputIdx + 1;
    return 0; /* null return */
    }
else 
    {
    *inputIdx = *inputIdx + 2;
    return GPSstrParse[idx];
    }
} /* gps_string_to_char */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		gps_conv_latitude_longitude                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Convert latitude and longitude from NMEA strings to the standard       *
*       format.                                                                *
*                                                                              *
* TEST:                                                                        *
*       test_GPS_parse provides coverage. If this function is                  *
*       updated, please re-run the test and update if necessary                *
*                                                                              *
*******************************************************************************/
void gps_conv_latitude_longitude( GPS_DATA* data ) 
{
/* Initialize variables */
uint8_t latitude_deg; /* Range: 0 to 90 */
uint8_t longitude_deg; /* Range: 0 to 180 */
float   latitude;
float   longitude;

/* Compute the degrees. Dividing by 100 drops the last two digits before the decimal.
   Truncate -- do not round */
latitude_deg = (uint8_t)(data->nmea_latitude / 100);
longitude_deg = (uint8_t)(data->nmea_longitude / 100);

/* Compute the minutes, divide by 100 to move after the decimal */
latitude = (data->nmea_latitude - (100 * latitude_deg)) / 60;
longitude = (data->nmea_longitude - (100 * longitude_deg)) / 60;

/* Add the degrees */
latitude += (float)latitude_deg;
longitude += (float)longitude_deg;

/* Compute the sign */
if (data->ns == 'S') {
    latitude = -latitude;
}
if (data->ew == 'W') {
    longitude = -longitude;
}

/* Set the values in the struct */
data->dec_latitude = latitude;
data->dec_longitude = longitude;
} /* gps_conv_latitude_longitude */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/