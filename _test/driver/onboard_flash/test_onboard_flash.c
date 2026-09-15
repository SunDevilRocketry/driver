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
#include <sys/mman.h>
#include <unistd.h>


/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "onboard_flash.h"
#include "error_sdr.h"

extern void      onboard_flash_test_fail_on_word(uint32_t word_number);
extern ERROR_CODE last_error_code;


/*------------------------------------------------------------------------------
Procedures: Helpers
------------------------------------------------------------------------------*/

static void set_up_vflash_buf(void) 
{
void*  flash_mapping;
void*  register_mapping;
uintptr_t register_region = FLASH_BASE_REG & ~( 0x10000UL - 1UL );

flash_mapping = mmap(   (void *)FLASH_BASE,
                        ONBOARD_FLASH_SIZE,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
                        -1,
                        0 );
register_mapping = mmap( (void *)register_region,
                        0x10000U,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
                        -1,
                        0 );

if ( flash_mapping == MAP_FAILED || register_mapping == MAP_FAILED )
    {
    perror( "mmap onboard flash test regions" );
    exit( EXIT_FAILURE );
    }

memset( flash_mapping, 0xA5, ONBOARD_FLASH_SIZE );
FLASH_SR1 = 0;
FLASH_CR1 = 0;
FLASH_ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN;

}

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
*       test_onboard_flash_write_addr                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       onboard_flash_write_addr tests.                                        *
*                                                                              *
*******************************************************************************/
void test_onboard_flash_write_addr
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
uint8_t expected_flash[ONBOARD_FLASH_SIZE];
uint8_t* flash = (uint8_t *)FLASH_BASE;

struct test_case cases[] =
    {
    { "Normal: Write one flashword at the start of flash",
      FLASH_BASE, ONBOARD_FLASH_WORD_SIZE },
    { "Normal: Write one flashword at the end of flash",
      FLASH_BASE + ONBOARD_FLASH_SIZE - ONBOARD_FLASH_WORD_SIZE,
      ONBOARD_FLASH_WORD_SIZE }
    };

for ( uint8_t test_num = 0;
      test_num < (uint8_t)( sizeof( cases ) / sizeof( struct test_case ) );
      test_num++ )
    {
    HAL_StatusTypeDef status;

    TEST_begin_nested_case( cases[test_num].description );

    memset( flash, 0xA5, ONBOARD_FLASH_SIZE );
    memset( scratch, 0x3C, sizeof( scratch ) );
    memcpy( expected_flash, flash, sizeof( expected_flash ) );
    memcpy( &expected_flash[cases[test_num].dst_addr - FLASH_BASE],
        scratch,
        sizeof( scratch ) );

    status = onboard_flash_write_addr( cases[test_num].dst_addr,
                                         scratch,
                                         cases[test_num].len );

    TEST_ASSERT_EQ_UINT( "Return HAL_OK when the write completes.",
                         (uint32_t)status,
                         (uint32_t)HAL_OK );

    TEST_ASSERT_EQ_MEMORY( "Preserve unrelated bytes and write the requested flashword.",
                flash,
                expected_flash,
                sizeof( expected_flash ) );
    TEST_ASSERT_EQ_UINT( "Leave instruction and data caches enabled after the write.",
                    FLASH_ACR & ( FLASH_ACR_ICEN | FLASH_ACR_DCEN ),
                    FLASH_ACR_ICEN | FLASH_ACR_DCEN );
    TEST_ASSERT_EQ_UINT( "Clear programming mode after each flashword.",
                    FLASH_CR1 & FLASH_CR1_PG,
                    0U );

    TEST_end_nested_case();
    }

} /* test_onboard_flash_write_addr */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_onboard_flash_write_addr_word_failure                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify a flashword programming error is returned and reported as a     *
*       fatal flash command error.                                             *
*                                                                              *
*******************************************************************************/
void test_onboard_flash_write_addr_word_failure
  (
  void
  )
{
uint8_t scratch[ONBOARD_FLASH_WORD_SIZE];
HAL_StatusTypeDef status;

TEST_begin_nested_case( "Robust: Return an error when a flashword write fails" );

memset( scratch, 0x3C, sizeof( scratch ) );
last_error_code = ERROR_UNKNOWN_FATAL_ERROR;
onboard_flash_test_fail_on_word( 0U );
status = onboard_flash_write_addr( FLASH_BASE,
                   scratch,
                   ONBOARD_FLASH_WORD_SIZE );

TEST_ASSERT_EQ_UINT( "Return HAL_ERROR when a flashword write reports an error.",
           (uint32_t)status,
           (uint32_t)HAL_ERROR );
TEST_ASSERT_EQ_UINT( "Report the flash command error through error_fail_fast().",
           (uint32_t)last_error_code,
           (uint32_t)ERROR_FLASH_CMD_ERROR );

TEST_end_nested_case();

} /* test_onboard_flash_write_addr_word_failure */


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
set_up_vflash_buf();

/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
    {
    { "onboard_flash_write_addr() precondition checks",
      test_onboard_flash_write_addr_preconditions },
    { "onboard_flash_write_addr() nominal case checks",
      test_onboard_flash_write_addr },
    { "onboard_flash_write_addr() word failure checks",
      test_onboard_flash_write_addr_word_failure }
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
