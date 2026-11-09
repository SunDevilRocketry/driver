/*******************************************************************************
*
* FILE: 
*      test_buzzer.c
*
* DESCRIPTION: 
*      Unit tests for functions in the buzzer module.
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

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       foo	          	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Example helper function for test								       *
*                                                                              *
*******************************************************************************/
int foo
	(
	int input
	) 
{
return input + 1;

} /* foo */


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/

/**
 * @brief Test the buzzer_beep function for proper HAL calls
 */
void test_buzzer_beep 
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_BUZZER_BEEP 3
printf("\nUnit Tests: test_bar\n");

/* Step: Set up test vectors (inputs, expected) */
int inputs[NUM_CASES_BUZZER_BEEP] = 
{
#include "cases/blank_inputs.txt"
};

int expected[NUM_CASES_BUZZER_BEEP] = 
{
#include "cases/blank_expected.txt"
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_BUZZER_BEEP; test_num++ )
	{
	/* Call function under test*/

	/* Check result*/
	TEST_ASSERT_EQ_INT( "Test that the result equals the expected", expected[test_num], foo(inputs[test_num]));
	}

} /* test_buzzer_beep */

/**
 * @brief Test the buzzer_multi_beeps function for proper HAL calls
 */
void test_buzzer_multi_beeps 
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_BUZZER_MULTI_BEEPS 3
printf("\nUnit Tests: test_bar\n");

/* Step: Set up test vectors (inputs, expected) */
int inputs[NUM_CASES_BUZZER_MULTI_BEEPS] = 
{
#include "cases/blank_inputs.txt"
};

int expected[NUM_CASES_BUZZER_MULTI_BEEPS] = 
{
#include "cases/blank_expected.txt"
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_BUZZER_MULTI_BEEPS; test_num++ )
	{
	/* Call function under test*/

	/* Check result*/
	TEST_ASSERT_EQ_INT( "Test that the result equals the expected", expected[test_num], foo(inputs[test_num]));
	}

} /* test_buzzer_multi_beeps */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main			                                   			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set up the testing enviroment, call tests, tear down the testing       *
*		environment															   *
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
	{ "buzzer_beep", test_buzzer_beep },
	{ "buzzer_multi_beeps", test_buzzer_multi_beeps }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "buzzer", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/