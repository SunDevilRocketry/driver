/**
  ******************************************************************************
  * @file           : mag.c
  * @brief          : Driver for rev 3's BMM350 magnetometer.
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  */

/* Includes ------------------------------------------------------------------*/

/* Project */
#include "mag.h"

/* Procedures ----------------------------------------------------------------*/

/**
 * @brief Initialize the magnetometer
 *
 * @details Restarts the magnetometer and applies the input config
 *
 * @retval The status of the magnetometer
 */
MAG_STATUS mag_init
    (
    MAG_CONFIG *mag_config_ptr
    )
{
// TODO actually implement
return MAG_OK;
}
