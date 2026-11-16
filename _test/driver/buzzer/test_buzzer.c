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
#include "buzzer.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
bool HAL_TIM_PWM_Start_called_correctly = false;
bool HAL_TIM_PWM_Stop_called_correctly = false;
bool HAL_Delay_called_correctly = false;
TIM_HandleTypeDef htim4;

/*------------------------------------------------------------------------------
Structs
------------------------------------------------------------------------------*/

struct TEST_CASE
{
	int buzz_duration;
	HAL_StatusTypeDef status1;
	HAL_StatusTypeDef status2;
} TEST_CASE;
struct RESULT
{
	BUZZ_STATUS status;
	bool HAL_TIM_PWM_Start_called_correctly;
	bool HAL_TIM_PWM_Stop_called_correctly;
	bool HAL_Delay_called_correctly;
} RESULT;

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

/*******************************************************************************
 *                                                                              *
 * PROCEDURE:                                                                   *
 *       test_buzzer_beep_helper	          	                                   *
 *                                                                              *
 * DESCRIPTION:                                                                 *
 *       Calls buzzer_beep given expected inputs and outputs in testing format  *
 *                                                                              *
 *******************************************************************************/
struct RESULT test_buzzer_beep_helper(
	struct TEST_CASE input)
{

	/* Set the results the mock HAL should return */
	MOCK_HAL_Status_Returns(input.status1, input.status2);

	BUZZ_STATUS buzz_result = buzzer_beep(input.buzz_duration);

	struct RESULT results;
	results.status = buzz_result;
	results.HAL_TIM_PWM_Start_called_correctly = HAL_TIM_PWM_Start_called_correctly;
	results.HAL_TIM_PWM_Stop_called_correctly = HAL_TIM_PWM_Stop_called_correctly;
	results.HAL_Delay_called_correctly = HAL_Delay_called_correctly;

	/* Reset booleans set by HAL mocks */
	HAL_TIM_PWM_Start_called_correctly = false;
    HAL_TIM_PWM_Stop_called_correctly = false;
	HAL_Delay_called_correctly = false;

	return results;

} /* test_buzzer_beep_helper */

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
	printf("\nUnit Tests: buzzer\n");

	/* Step: Set up test vectors (inputs, expected) */
	struct TEST_CASE inputs[NUM_CASES_BUZZER_BEEP] =
		{
			/* { BUZZ_DURATION, HAL_STATUS_1_TO_RETURN, HAL_STATUS_2_TO_RETURN }*/
			{500, HAL_OK, HAL_OK},
			{500, HAL_OK, HAL_ERROR},
			{500, HAL_ERROR, HAL_ERROR}

	};

	struct RESULT expected[NUM_CASES_BUZZER_BEEP] =
		{
			/* { BUZZ_RESULT, HAL_TIM_PWM_Start_called_correctly, HAL_TIM_PWM_Stop_called_correctly, HAL_Delay_called_correctly }*/
			{BUZZ_OK, true, true, true},
			{BUZZ_HAL_ERROR, true, true, true},
			{BUZZ_HAL_ERROR, true, false, false}
		};

	/* Step: Execute tests */
	for (int test_num = 0; test_num < NUM_CASES_BUZZER_BEEP; test_num++)
	{

		/* Call function under test*/
		struct RESULT testResult = test_buzzer_beep_helper(inputs[test_num]);
		/* Check results*/

		TEST_ASSERT_EQ_SINT("Test that the returned BUZZ_STATUS equals the expected", testResult.status, expected[test_num].status);

		/* Check that HAL_TIM_PWM_Start was called, or not called, correctly*/
		if (expected[test_num].HAL_TIM_PWM_Start_called_correctly)
		{
			TEST_ASSERT_TRUE("Test that HAL_TIM_PWM_Start was called with the correct arguments", testResult.HAL_TIM_PWM_Start_called_correctly);
		}
		else
		{
			TEST_ASSERT_FALSE("Test that HAL_TIM_PWM_Start was not called", testResult.HAL_TIM_PWM_Start_called_correctly);
		}

		/* Check that HAL_TIM_PWM_Stop was called, or not called, correctly*/
		if (expected[test_num].HAL_TIM_PWM_Stop_called_correctly)
		{
			TEST_ASSERT_TRUE("Test that HAL_TIM_PWM_Stop was called with the correct arguments", testResult.HAL_TIM_PWM_Stop_called_correctly);
		}
		else
		{
			TEST_ASSERT_FALSE("Test that HAL_TIM_PWM_Stop was not called", testResult.HAL_TIM_PWM_Stop_called_correctly);
		}

		/* Check that HAL_Delay was called, or not called, correctly*/
		if (expected[test_num].HAL_TIM_PWM_Stop_called_correctly)
		{
			TEST_ASSERT_TRUE("Test that HAL_Delay was called with the correct arguments", testResult.HAL_Delay_called_correctly);
		}
		else
		{
			TEST_ASSERT_FALSE("Test that HAL_Delay was not called", testResult.HAL_Delay_called_correctly);
		}

	}

} /* test_buzzer_beep */

/**
 * @brief Test the buzzer_multi_beeps function for proper HAL calls
 */
void test_buzzer_multi_beeps(
	void)
{
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
int main(
	void)
{
	/*------------------------------------------------------------------------------
	Test Cases
	------------------------------------------------------------------------------*/
	unit_test tests[] =
		{
			{"buzzer_beep", test_buzzer_beep},
			{"buzzer_multi_beeps", test_buzzer_multi_beeps}};

	/*------------------------------------------------------------------------------
	Call the framework
	------------------------------------------------------------------------------*/
	TEST_INITIALIZE_TEST("buzzer", tests);

} /* main */

/*******************************************************************************
 * END OF FILE                                                                  *
 *******************************************************************************/