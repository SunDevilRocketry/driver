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
 Macros 
------------------------------------------------------------------------------*/

#ifdef A0002_REV2
/* I2C Device Params */
#define BARO_I2C_ADDR	    ( 0x76 << 1 )	/* 1110110 -> 0x76 */


/* Barometric Pressure Sensor register addresses */
#define BARO_REG_CHIP_ID        ( 0x00 )
#define BARO_REG_ERR_REG        ( 0x02 )
#define BARO_REG_PRESS_DATA     ( 0x04 )  
#define BARO_REG_TEMP_DATA      ( 0x07 ) 
#define BARO_REG_PWR_CTRL       ( 0x1B )
#define BARO_REG_OSR            ( 0x1C ) 
#define BARO_REG_ODR            ( 0x1D )
#define BARO_REG_CONFIG         ( 0x1F )
#define BARO_REG_NVM_PAR_T1     ( 0x31 )
#define BARO_REG_NVM_PAR_T2     ( 0x33 )
#define BARO_REG_NVM_PAR_T3     ( 0x35 )
#define BARO_REG_NVM_PAR_P1     ( 0x36 )
#define BARO_REG_NVM_PAR_P2     ( 0x38 )
#define BARO_REG_NVM_PAR_P3     ( 0x3A )
#define BARO_REG_NVM_PAR_P4     ( 0x3B )
#define BARO_REG_NVM_PAR_P5     ( 0x3C )
#define BARO_REG_NVM_PAR_P6     ( 0x3E )
#define BARO_REG_NVM_PAR_P7     ( 0x40 )
#define BARO_REG_NVM_PAR_P8     ( 0x41 )
#define BARO_REG_NVM_PAR_P9     ( 0x42 )
#define BARO_REG_NVM_PAR_P10    ( 0x44 )
#define BARO_REG_NVM_PAR_P11    ( 0x45 )
#define BARO_REG_CMD            ( 0x7E )

/* Baro device id */
#define BMP390_DEVICE_ID        ( 0x60 )
#define BMP388_DEVICE_ID        ( 0x50 )

/* Size of calibration data in bytes */
#define BARO_CAL_BUFFER_SIZE    ( 21   )

/* Default I2C HAL timeout */
#ifndef SDR_DEBUG
	#define BARO_DEFAULT_TIMEOUT    ( 1    )
#else
	#define BARO_DEFAULT_TIMEOUT    ( 0xFFFFFFFF )
#endif /* ifndef SDR_DEBUG */

/* Baro sensor command codes */
#define BARO_CMD_RESET          ( 0xB6 )
#define BARO_CMD_FIFO_FLUSH     ( 0xB0 )
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
	#ifdef A00100               ,
	/* If the CRC check on r3 baro memory fails */
	BARO_INVALID_PROM,
	#endif
	BARO_ERROR                  ,
	BARO_CAL_ERROR              ,
	#ifdef A0002_REV2
	BARO_I2C_ERROR              ,
	#endif
	#ifdef A0010
	BARO_SPI_ERROR              ,
	#endif
	BARO_CANNOT_RESET           ,
	BARO_FIFO_ERROR				,
	BARO_BUSY
	} BARO_STATUS;

/* Sensor enable encodings */
typedef enum _BARO_SENSOR_ENABLES
	{
	BARO_DISABLED           = 0,
    BARO_PRESS_ENABLED      = 1,
	BARO_TEMP_ENABLED       = 2,
	BARO_PRESS_TEMP_ENABLED = 3
	} BARO_SENSOR_ENABLES;

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

	#ifdef A0002_REV2
	/* Operating mode */
	BARO_MODE mode;
	#endif

	/* Pressure Oversampling setting  */
	BARO_PRESS_OSR_SETTING press_OSR_setting;

	/* Temperature Oversampling setting */
	BARO_TEMP_OSR_SETTING temp_OSR_setting;

	#ifdef A0002_REV2
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

#ifdef A0010
/* verifies sensor can be accessed */
/* The serial is 12 bits on the rev 3 sensor */
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id 
   	uint16_t* baro_id
	);
#endif

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

BARO_STATUS start_baro_read_IT();
BARO_STATUS baro_IT_handler();
BARO_STATUS get_baro_it(float* pres_ptr, float* temp_ptr);


#ifdef __cplusplus
}
#endif
#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
