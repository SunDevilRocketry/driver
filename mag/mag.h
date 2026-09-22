/*******************************************************************************
*
* FILE: 
* 		mag.h
*
* DESCRIPTION: 
* 		Contains API functions for A0010 (rev3 Flight Computer) magnetometer
*
* COPYRIGHT:                                                                   
*       Copyright (c) 2026 Sun Devil Rocketry.                                 
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

/* How it works, studying the BMM350 driver:
 * - You read the 32 byte OTP ROM on startup
 * - You do an I2C burst read of X, Y, Z, and temp (used for compensation)
 * - You read compensation coefficents from the OTP ROM
 * - You apply them and pass back the result
 */
