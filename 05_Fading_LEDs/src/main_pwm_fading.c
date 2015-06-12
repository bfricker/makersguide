
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#define LED_PORT	gpioPortE
#define LED_PIN		2

#define DEBUG_BREAK 			__asm__("BKPT #0");
#define ONE_MILLISECOND_BASE_VALUE_COUNT 	1000
#define ONE_SECOND_TIMER_COUNT				13672
#define MILLISECOND_DIVISOR					13.672

uint64_t base_value = 0;
bool timer1_overflow = false;

#define TIMER_TOP				100
#define TIMER_CHANNEL			2

#define RAMP_UP_TIME_MS			500
#define RAMP_DOWN_TIME_MS		700
#define HIGH_DURATION_MS		1000
#define LOW_DURATION_MS			500
#define MAX_BRIGHTNESS			100
#define MIN_BRIGHTNESS			0

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

int main(void)
{
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_TIMER3, true);

	// Set up TIMER1 for timekeeping
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER1 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0) ;

	// Enable LED output
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

	// Create the object initializer for LED PWM
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.cmoa = timerOutputActionToggle;

	// Configure TIMER3 CC channel 2
	TIMER_InitCC(TIMER3, TIMER_CHANNEL, &timerCCInit);

	// Route CC2 to location 1 (PE3) and enable pin for cc2
	TIMER3->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC1);

	// Set Top Value
	TIMER_TopSet(TIMER3, TIMER_TOP);

	// Set the PWM duty cycle here!
	TIMER_CompareBufSet(TIMER3, TIMER_CHANNEL, 0);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit2 = TIMER_INIT_DEFAULT;
	timerInit2.prescale = timerPrescale256;

	TIMER_Init(TIMER3, &timerInit2);

	enum mode_values { RAMPING_UP, HIGH, RAMPING_DOWN, LOW};

	// Check for properly sized constants
	uint16_t delta = MAX_BRIGHTNESS - MIN_BRIGHTNESS;
	if ( delta == 0 || RAMP_UP_TIME_MS % delta || RAMP_DOWN_TIME_MS % delta)
	{
		DEBUG_BREAK
	}

	// Set the initial condition
	uint16_t mode = RAMPING_UP;
	uint32_t time_step = RAMP_UP_TIME_MS / delta;
	uint16_t brightness = MIN_BRIGHTNESS;
	TIMER_CompareBufSet(TIMER3, TIMER_CHANNEL, brightness);

	uint64_t mode_timeout = set_ms_timeout(RAMP_UP_TIME_MS);

	while (1)
	{
		switch (mode)
		{
			case RAMPING_UP:
				delay_ms(time_step);
				brightness++;
				TIMER_CompareBufSet(TIMER3, TIMER_CHANNEL, brightness);
				if (expired_ms(mode_timeout))
				{
					mode = HIGH;
					mode_timeout = set_ms_timeout(HIGH_DURATION_MS);
				}
				break;
			case HIGH:
				if (expired_ms(mode_timeout))
				{
					mode = RAMPING_DOWN;
					time_step = RAMP_DOWN_TIME_MS / delta;
					mode_timeout = set_ms_timeout(RAMP_DOWN_TIME_MS);
				}
				break;
			case RAMPING_DOWN:
				delay_ms(time_step);
				brightness--;
				TIMER_CompareBufSet(TIMER3, TIMER_CHANNEL, brightness);
				if (expired_ms(mode_timeout))
				{
					mode = LOW;
					mode_timeout = set_ms_timeout(LOW_DURATION_MS);
				}
				break;
			case LOW:
				if (expired_ms(mode_timeout))
				{
					mode = RAMPING_UP;
					time_step = RAMP_UP_TIME_MS / delta;
					mode_timeout = set_ms_timeout(RAMP_UP_TIME_MS);
				}
				break;
		}
	}
}

