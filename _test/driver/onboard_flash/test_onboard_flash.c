/*******************************************************************************
*
* FILE:
*      test_onboard_flash.c
*
* DESCRIPTION:
*      Unit tests for driver/onboard_flash/onboard_flash.c (precondition checks
*      on host gcc). Paths that program flash are not exercised here because
*      they require RAM-backed FLASH_BASE and register modeling.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "onboard_flash.h"


/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_onboard_flash_write_addr_preconditions                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       onboard_flash_write_addr must reject non-flashword-aligned length or   *
*       destination address before touching the flash controller.             *
*                                                                              *
*******************************************************************************/
void test_onboard_flash_write_addr_preconditions
    (
    void
    )
{
struct test_case
    {
    const char* description;
    uint32_t    dst_addr;
    uint32_t    len;
    };
uint8_t scratch[ONBOARD_FLASH_WORD_SIZE];

struct test_case cases[] =
    {
    { "Robust: length 1 is not a multiple of flashword size", FLASH_BASE, 1U },
    { "Robust: length 31 is not a multiple of flashword size", FLASH_BASE, 31U },
    { "Robust: length 33 is not a multiple of flashword size", FLASH_BASE, 33U },
    { "Robust: destination +4 from flash base is not 32-byte aligned",
      FLASH_BASE + 4U, ONBOARD_FLASH_WORD_SIZE },
    { "Robust: destination +31 from flash base is not 32-byte aligned",
      FLASH_BASE + 31U, ONBOARD_FLASH_WORD_SIZE }
    };

for ( uint8_t test_num = 0;
      test_num < (uint8_t)( sizeof( cases ) / sizeof( struct test_case ) );
      test_num++ )
    {
    HAL_StatusTypeDef status;

    TEST_begin_nested_case( cases[test_num].description );

    memset( scratch, 0, sizeof( scratch ) );
    status = onboard_flash_write_addr( cases[test_num].dst_addr,
                                         scratch,
                                         cases[test_num].len );

    TEST_ASSERT_EQ_UINT( "Return HAL_ERROR when preconditions are violated.",
                         (uint32_t)status,
                         (uint32_t)HAL_ERROR );

    TEST_end_nested_case();
    }

} /* test_onboard_flash_write_addr_preconditions */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment, call tests, tear down the testing      *
*       environment.                                                            *
*                                                                              *
*******************************************************************************/
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
    { "onboard_flash_write_addr() precondition checks",
      test_onboard_flash_write_addr_preconditions }
    };

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "onboard_flash.c", tests );

return 0;

} /* main */


/*******************************************************************************
* END OF FILE
*******************************************************************************/
