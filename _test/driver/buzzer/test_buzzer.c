 /**
  ********************************************************************************
  * @file test_buzzer.c
  * @brief Unit tests for functions in the buzzer module.
  *******************************************************************************
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
  *
  ******************************************************************************/

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
#include "buzzer.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
bool HAL_TIM_PWM_Start_called_correctly = false;
bool HAL_TIM_PWM_Stop_called_correctly = false;
int delayCalls[] = {-1, -1, -1, -1, -1};
TIM_HandleTypeDef htim4;

/*------------------------------------------------------------------------------
Structs
------------------------------------------------------------------------------*/

struct TEST_CASE
{
	int buzz_duration;
	int wait_duration;
	HAL_StatusTypeDef status1;
	HAL_StatusTypeDef status2;
	HAL_StatusTypeDef status3;
	HAL_StatusTypeDef status4;
} TEST_CASE;
struct RESULT
{
	BUZZ_STATUS status;
	bool HAL_TIM_PWM_Start_called_correctly;
	bool HAL_TIM_PWM_Stop_called_correctly;
	int delayCalls[5];
	int numberOfDelayCalls;
} RESULT;

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

/**
 * @brief Calls buzzer_beep given expected inputs and outputs in testing format
 *   *
 * @param input A test case for the helper to execute
 * 
 * @return The results of the test
 */
struct RESULT test_buzzer_beep_helper
(
	struct TEST_CASE input
)
{
/* Set the results the mock HAL should return */
MOCK_HAL_Setup(input.status1, input.status2, HAL_OK, HAL_OK );

BUZZ_STATUS buzz_result = buzzer_beep(input.buzz_duration);

struct RESULT results;
results.status = buzz_result;
results.HAL_TIM_PWM_Start_called_correctly = HAL_TIM_PWM_Start_called_correctly;
results.HAL_TIM_PWM_Stop_called_correctly = HAL_TIM_PWM_Stop_called_correctly;
for ( int i = 0; i<5; i++ )
	{
	results.delayCalls[i] = delayCalls[i];
	}

/* Reset values set by HAL mocks */
HAL_TIM_PWM_Start_called_correctly = false;
HAL_TIM_PWM_Stop_called_correctly = false;
for ( int i = 0; i<5; i++ )
	{
	delayCalls[i] = -1;
	}

return results;
} /* test_buzzer_beep_helper */


/**
 * @brief Calls buzzer_multi_beeps given expected inputs and outputs in testing format
 *   *
 * @param input A test case for the helper to execute
 * 
 * @return The results of the test
 */
struct RESULT test_buzzer_multi_beeps_helper
(
	struct TEST_CASE input
)
{
/* Set the results the mock HAL should return */
MOCK_HAL_Setup(input.status1, input.status2, input.status3, input.status4 );

BUZZ_STATUS buzz_result = buzzer_multi_beeps(input.buzz_duration, input.wait_duration, 2);

struct RESULT results;
results.status = buzz_result;
results.HAL_TIM_PWM_Start_called_correctly = HAL_TIM_PWM_Start_called_correctly;
results.HAL_TIM_PWM_Stop_called_correctly = HAL_TIM_PWM_Stop_called_correctly;

for ( int i = 0; i<5; i++ )
	{
	results.delayCalls[i] = delayCalls[i];
	}

/* Reset values set by HAL mocks */
HAL_TIM_PWM_Start_called_correctly = false;
HAL_TIM_PWM_Stop_called_correctly = false;
for ( int i = 0; i<5; i++ )
	{
	delayCalls[i] = -1;
	}

return results;
} /* test_buzzer_multi_beeps_helper */


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
printf("\nUnit Tests: buzzer_beep\n");

/* Step: Set up test vectors (inputs, expected) */
struct TEST_CASE inputs[NUM_CASES_BUZZER_BEEP] =
	{
		/* { BUZZ_DURATION, WAIT_DURATION, HAL_STATUS_1_TO_RETURN, HAL_STATUS_2_TO_RETURN, HAL_STATUS_3_TO_RETURN, HAL_STATUS_4_TO_RETURN }*/
		/* Note: the third and fourth HAL return status and the WAIT_DURATION are irrelevant for the buzzer_beep tests, they are used for the buzzer_multi_beeps tests */
		{500, 0, HAL_OK, HAL_OK, HAL_OK, HAL_OK},
		{500, 0, HAL_OK, HAL_ERROR, HAL_OK, HAL_OK},
		{500, 0, HAL_ERROR, HAL_ERROR, HAL_OK, HAL_OK}

	};

struct RESULT expected[NUM_CASES_BUZZER_BEEP] =
	{
		/* { BUZZ_RESULT, HAL_TIM_PWM_Start_called_correctly, HAL_TIM_PWM_Stop_called_correctly, delayCalls }*/
		{BUZZ_OK, true, true, { 500, -1, -1, -1, -1} },
		{BUZZ_HAL_ERROR, true, true, { 500, -1, -1, -1, -1}},
		{BUZZ_HAL_ERROR, true, false, { -1, -1, -1, -1, -1}}
	};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_BUZZER_BEEP; test_num++ )
	{

	/* Call function under test*/
	struct RESULT testResult = test_buzzer_beep_helper(inputs[test_num]);
	/* Check results*/

	TEST_ASSERT_EQ_SINT("Test that the returned BUZZ_STATUS equals the expected", testResult.status, expected[test_num].status);

	/* Check that HAL_TIM_PWM_Start was called, or not called, correctly*/
	if ( expected[test_num].HAL_TIM_PWM_Start_called_correctly )
		{
		TEST_ASSERT_TRUE("Test that HAL_TIM_PWM_Start was called with the correct arguments", testResult.HAL_TIM_PWM_Start_called_correctly);
		}
	else
		{
		TEST_ASSERT_FALSE("Test that HAL_TIM_PWM_Start was not called", testResult.HAL_TIM_PWM_Start_called_correctly);
		}

	/* Check that HAL_TIM_PWM_Stop was called, or not called, correctly*/
	if ( expected[test_num].HAL_TIM_PWM_Stop_called_correctly )
		{
		TEST_ASSERT_TRUE("Test that HAL_TIM_PWM_Stop was called with the correct arguments", testResult.HAL_TIM_PWM_Stop_called_correctly);
		}
	else
		{
		TEST_ASSERT_FALSE("Test that HAL_TIM_PWM_Stop was not called", testResult.HAL_TIM_PWM_Stop_called_correctly);
		}

	/* Check that the delays executed are what was expected*/
	for ( int i = 0; i<5; i++ )
		{
		TEST_ASSERT_EQ_SINT("Test that HAL_Delay was called the expected number of times with the expected arguments", testResult.delayCalls[i], expected[test_num].delayCalls[i]);
		}

}
} /* test_buzzer_multi_beeps */


/**
 * @brief Test the buzzer_multi_beeps function for proper HAL calls
 */
void test_buzzer_multi_beeps
(
	void
)
{
/* Step: Set up test */
#define NUM_CASES_buzzer_multi_beeps 2
printf("\nUnit Tests: buzzer_multi_beeps\n");

/* Step: Set up test vectors (inputs, expected) */
struct TEST_CASE inputs[NUM_CASES_buzzer_multi_beeps] =
	{
		/* { BUZZ_DURATION, WAIT_DURATION, HAL_STATUS_1_TO_RETURN, HAL_STATUS_2_TO_RETURN, HAL_STATUS_3_TO_RETURN }*/
		{300, 50, HAL_OK, HAL_OK, HAL_OK, HAL_OK},
		{300, 50, HAL_OK, HAL_OK, HAL_OK, HAL_ERROR},

};

struct RESULT expected[NUM_CASES_buzzer_multi_beeps] =
	{
		/* { BUZZ_RESULT, HAL_TIM_PWM_Start_called_correctly, HAL_TIM_PWM_Stop_called_correctly, delayCalls }*/
		{BUZZ_OK, true, true, { 300, 50, 300, 50, -1} },
		{BUZZ_HAL_ERROR, true, true, { 300, 50, 300, -1, -1}}
	};

/* Step: Execute tests */
for (int test_num = 0; test_num < NUM_CASES_buzzer_multi_beeps; test_num++)
	{

	/* Call function under test*/
	struct RESULT testResult = test_buzzer_multi_beeps_helper(inputs[test_num]);
	/* Check results*/

	TEST_ASSERT_EQ_SINT("Test that the returned BUZZ_STATUS equals the expected", testResult.status, expected[test_num].status);

	/* Check that the delays executed are what was expected*/
	for ( int i = 0; i<5; i++ )
		{
		TEST_ASSERT_EQ_SINT("Test that HAL_Delay was called the expected number of times with the expected arguments", testResult.delayCalls[i], expected[test_num].delayCalls[i]);
		}

	}

} /* test_buzzer_multi_beeps */


/**
 * @brief Set up the testing enviroment, call tests, tear down the testing environment
 *
 * @return Return status
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
		{"buzzer_beep", test_buzzer_beep},
		{"buzzer_multi_beeps", test_buzzer_multi_beeps}
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST("buzzer", tests);
} /* main */