/*******************************************************************************
*
* FILE: 
* 		baro.h
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
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
#include <stdbool.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BARO_H 
#define BARO_H 

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#ifdef A0010
#include "sdr_pin_defines_A0010.h"
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return codes for API functions */
typedef enum _BARO_STATUS
	{
	BARO_OK                   =0,
	BARO_FAIL                   , 
	BARO_TIMEOUT                ,
	BARO_UNRECOGNIZED_HAL_STATUS,
	BARO_UNSUPPORTED_CONFIG     ,
	BARO_UNRECOGNIZED_CHIP_ID   ,
	BARO_ERROR                  ,
	BARO_CAL_ERROR              ,
	BARO_I2C_ERROR              ,
	BARO_SPI_ERROR              ,
	BARO_CANNOT_RESET           ,
	BARO_FIFO_ERROR				,
	BARO_BUSY                   ,
	BARO_INVALID_PROM 	/* If the CRC check on r3 baro memory fails */
	} BARO_STATUS;

/* Sensor enable encodings */
typedef enum _BARO_SENSOR_ENABLES
	{
	BARO_DISABLED           = 0,
    BARO_PRESS_ENABLED      = 1,
	BARO_TEMP_ENABLED       = 2,
	BARO_PRESS_TEMP_ENABLED = 3
	} BARO_SENSOR_ENABLES;

#ifdef A0010
/* Interrupt events for R3 version of driver.
 * Takes similar approach to telemetry in mod.
 * Events are in order of when they occure.
 * Some events occur multiple times per 
 * data acquisition cycle.
 */
typedef enum _BARO_EVENT 
    {
    BARO_EVENT_START_READ = 0,
    BARO_EVENT_TX_CPLT,
    BARO_EVENT_DELAY_ELAPSED,
    BARO_EVENT_TXRX_CPLT
    } BARO_EVENT;
#endif

#ifdef A0002_REV2
/* Operating mode of baro sensor */
typedef enum _BARO_MODE
	{
	BARO_SLEEP_MODE  = 0,
	BARO_FORCED_MODE = 1,
	BARO_NORMAL_MODE = 3
	} BARO_MODE;
#endif
	
/* Pressure Sensor Oversampling Settings */
typedef enum _BARO_PRESS_OSR_SETTING
	{
	#ifdef A0002_REV2
	BARO_PRESS_OSR_X1 = 0,
    BARO_PRESS_OSR_X2    ,
	BARO_PRESS_OSR_X4    ,
	BARO_PRESS_OSR_X8    ,
	BARO_PRESS_OSR_X16   ,
	BARO_PRESS_OSR_X32
	#endif
	/* Prevent any possibility that vals get misinterpreted between revs */
	#ifdef A0010
	BARO_PRESS_OSR_X256 = 6,
	BARO_PRESS_OSR_X512    ,
	BARO_PRESS_OSR_X1024   ,
	BARO_PRESS_OSR_X2048   ,
	BARO_PRESS_OSR_X4096
	#endif
	} BARO_PRESS_OSR_SETTING;

/* Temperature Sensor Oversampling Settings */
typedef enum _BARO_TEMP_OSR_SETTING
	{
	#ifdef A0002_REV2
	BARO_TEMP_OSR_X1 = 0,
    BARO_TEMP_OSR_X2    ,
	BARO_TEMP_OSR_X4    ,
	BARO_TEMP_OSR_X8    ,
	BARO_TEMP_OSR_X16   ,
	BARO_TEMP_OSR_X32
	#endif
	/* Prevent any possibility that vals get misinterpreted between revs */
	#ifdef A0010
	BARO_TEMP_OSR_X256 = 6,
	BARO_TEMP_OSR_X512    ,
	BARO_TEMP_OSR_X1024   ,
	BARO_TEMP_OSR_X2048   ,
	BARO_TEMP_OSR_X4096
	#endif
	} BARO_TEMP_OSR_SETTING;


#ifdef A0002_REV2
/* Sample Frequency settings */
typedef enum _BARO_ODR_SETTING
	{
	BARO_ODR_200HZ = 0,
	BARO_ODR_100HZ    ,
	BARO_ODR_50HZ     ,
	BARO_ODR_25HZ     ,
	BARO_ODR_25_2HZ   ,
	BARO_ODR_25_4HZ   ,
	BARO_ODR_25_8HZ   ,
	BARO_ODR_25_16HZ  ,
	BARO_ODR_25_32HZ
	} BARO_ODR_SETTING;

/* IIR Filter coefficient selection */
typedef enum _BARO_IIR_SETTING
	{
	BARO_IIR_COEF_0   = 0,
	BARO_IIR_COEF_1   = 1,
	BARO_IIR_COEF_3   = 2,
	BARO_IIR_COEF_7   = 3,
	BARO_IIR_COEF_15  = 4,
	BARO_IIR_COEF_31  = 5,
	BARO_IIR_COEF_63  = 6,
	BARO_IIR_COEF_127 = 7 
	} BARO_IIR_SETTING;
#endif
	
/* Baro sensor configuration settings struct */
typedef struct _BARO_CONFIG
	{
	/* Sensor enables */
	BARO_SENSOR_ENABLES enable;

	/* Pressure Oversampling setting  */
	BARO_PRESS_OSR_SETTING press_OSR_setting;

	/* Temperature Oversampling setting */
	BARO_TEMP_OSR_SETTING temp_OSR_setting;

	#ifdef A0002_REV2
	/* Operating mode */
	BARO_MODE mode;
	
	/* Sampling frequency */
	BARO_ODR_SETTING ODR_setting;

	/* IIR Filter Coefficient Selection */
	BARO_IIR_SETTING IIR_setting;
	#endif
	} BARO_CONFIG;


/* These structs are static to the rev3 implementation.
* They should be for rev2 as well, but aren't.
* I'm leaving it this way for rev2 code because emulator
* depends on it.
*/
#ifdef A0002_REV2
/* Baro calibration data struct in integer format */
typedef struct _BARO_CAL_DATA_INT
	{
	/* Temperature Compensation Coefficients */
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t   par_t3;

	/* Pressure Compensation Coefficients */
	int16_t  par_p1;
	int16_t  par_p2;
	int8_t   par_p3;
	int8_t   par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t   par_p7;
	int8_t   par_p8;
	int16_t  par_p9;
	int8_t   par_p10;
	int8_t   par_p11;
	} BARO_CAL_DATA_INT;

/* Baro calibration data struct in floating point format */
typedef struct _BARO_CAL_DATA
	{
	/* Temperature Compensation Coefficients */
	float par_t1;
	float par_t2;
	float par_t3;

	/* Pressure Compensation Coefficients */
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;

	/* Temperature Compensated Measurement, for pressure compensation */
	float comp_temp;

	} BARO_CAL_DATA;
#endif

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Intialize the barometric pressure sensor */
BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	);

#ifdef A0002_REV2
/* verifies sensor can be accessed */
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id 
	);
#endif
/* The above is redundant on rev3, as we have a CRC check */
/* Thus, we don't implement it. */

/* Blocking implementations are not supported on r3 */
#ifdef A0002_REV2
/* gets pressure data from sensor */
BARO_STATUS baro_get_pressure
	(
    float* pressure_ptr 
	);

/* gets temp data from sensor */
BARO_STATUS baro_get_temp
	(
    float* temp_ptr 
	);

/* converts pressure and temp data into altitude --> do research on formula */
BARO_STATUS baro_get_altitude
	(
    void
	);
#endif
	
/* returns the baro_data_ready flag */
bool baro_get_baro_data_ready
    (
    void 
    );

BARO_STATUS baro_get_IT
    (
    float* pres_ptr,
    float* temp_ptr
    );

BARO_STATUS baro_start_read_IT
    (
    void
    );

#ifdef A0002_REV2
BARO_STATUS baro_IT_handler
    (
    void
    );
#endif
#ifdef A0010
/* R3 baro has more complicated data acquisition sequence.
 * We thus need to prevent various race conditions;
 * this required making a breaking API change.
 * The approach is very similar to the telemetry module.
 */
BARO_STATUS baro_IT_handler
    (
    BARO_EVENT update_cause
    );
#endif


#ifdef __cplusplus
}
#endif
#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
