/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

//#ifdef FOO

#define LED_PORT	gpioPortE
#define LED_PIN		2

#define DEBUG_BREAK 						__asm__("BKPT #0");
#define ONE_MILLISECOND_BASE_VALUE_COUNT 	1000
#define ONE_SECOND_TIMER_COUNT				13672
#define MILLISECOND_DIVISOR					13.672

volatile uint64_t base_value = 0;
volatile bool timer1_overflow = false;
volatile uint32_t msTicks = 0;

void TIMER1_IRQHandler(void)
{
	timer1_overflow = true;
	base_value += ONE_MILLISECOND_BASE_VALUE_COUNT;
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
}

uint64_t get_time_in_ms()
{
	// Clear our overflow indicator
	timer1_overflow = false;

	// Get the current count
	uint16_t count = TIMER1->CNT;

	// Get a copy of the base value
	uint64_t copy_of_base_value = base_value;

	// Now check to make sure that the ISR didn't fire while in here
	// If it did, then grab the values again
	if (timer1_overflow)
	{
		count = TIMER1->CNT;
		copy_of_base_value = base_value;
	}

	// Now calculate the number of milliseconds the program has run
	return copy_of_base_value + count / MILLISECOND_DIVISOR;
}

void Delay(uint32_t dlyTicks)
{
	uint32_t curTicks;

	curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks) ;
}

void delay_ms(uint64_t milliseconds)
{
	uint64_t trigger_time = get_time_in_ms() + milliseconds;
	while (get_time_in_ms() < trigger_time)
		;
}

// Pass in the ms_ticks when the timer started, get back the number of ms since then
// Pass in a future value to get time until timeout occurs, a neg value
int64_t elapsed_ms(int64_t start_ms)
{
	return get_time_in_ms() - start_ms;
}

// Pass in the elapsed time before it times out, returns the future time
int64_t set_ms_timeout(int64_t timeout_ms)
{
	return get_time_in_ms() + timeout_ms;
}

// Check to see if the future time has elapsed.
int64_t expired_ms(int64_t timeout_ms)
{
	if (timeout_ms < get_time_in_ms())
	{
		return true;
	}
	return false;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/

int main(void)
{
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER1 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0)

	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

	uint64_t timer = get_time_in_ms();
	for (int i= 0; i < 10000; i++)
	{
		GPIO->P[LED_PORT].DOUTSET = 1 << LED_PIN;
		GPIO->P[LED_PORT].DOUTCLR = 1 << LED_PIN;
	}
	timer = elapsed_ms(timer);

	while (1)
		;
}
//int main(void)
//{
//	CHIP_Init();
//
//	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
//	{
//		DEBUG_BREAK;
//	}
//
//	CMU_ClockEnable(cmuClock_TIMER1, true);
//
//	// Create a timerInit object, based on the API default
//	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
//	timerInit.prescale = timerPrescale1024;
//
//	TIMER_IntEnable(TIMER1, TIMER_IF_OF);
//
//	// Enable TIMER1 interrupt vector in NVIC
//	NVIC_EnableIRQ(TIMER1_IRQn);
//
//	// Set TIMER Top value
//	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);
//
//	TIMER_Init(TIMER1, &timerInit);
//
//	// Wait for the timer to get going
//	while (TIMER1->CNT == 0)
//		;
//
//	// Code to use timer goes here
//
//	int64_t start_time = get_time_in_ms();
//
//	// Set a timeout of 1 second
//	int64_t timeout = set_ms_timeout(1000);
//
//	while (!expired_ms(timeout))
//	{
//		// You can do work in here while you wait
//		// That makes this better than a simple delay
//	}
//
//	int32_t end_time = elapsed_ms(start_time);
//
//	if (end_time != 1000 && end_time != 1001)
//	{
//		DEBUG_BREAK
//	}
//
//	Delay(1000);
//
//	uint16_t random;
//	int32_t last = get_time_in_ms();
//	while (1)
//	{
//		int32_t duration = get_time_in_ms() - last;
//		if (duration < 0 || duration >  110)
//		{
//			DEBUG_BREAK;
//		}
//		last = get_time_in_ms();
//		random = rand() % 100;
//		Delay(random);
//	}
//}

//int main(void)
//{
//	// Chip errata
//    CHIP_Init();
//
//	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
//	{
//		DEBUG_BREAK;
//	}
//
//    while (1)
//    {
//    	Delay(10000);
//    	// Do something that will happen every second
//    }
//}

void SysTick_Handler(void)
{
	msTicks++;
}

//#endif
