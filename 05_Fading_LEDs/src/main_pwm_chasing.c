
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#define LED_PORT	gpioPortD
#define LED_PIN0	1
#define LED_PIN1	2
#define LED_PIN2	3

#define DEBUG_BREAK 						__asm__("BKPT #0");
#define ONE_MILLISECOND_BASE_VALUE_COUNT 	1000
#define ONE_SECOND_TIMER_COUNT				13672
#define MILLISECOND_DIVISOR					13.672

uint64_t base_value = 0;
bool timer1_overflow = false;

#define TIMER_TOP				100
#define TIMER_CHANNEL			2

#define SPEED_MULTIPLIER		5
#define RAMP_UP_TIME_MS			100 * SPEED_MULTIPLIER
#define RAMP_DOWN_TIME_MS		100 * SPEED_MULTIPLIER
#define HIGH_DURATION_MS		100 * SPEED_MULTIPLIER
#define LOW_DURATION_MS			0
#define MAX_BRIGHTNESS			100
#define MIN_BRIGHTNESS			0

#define OVERLAP					125 * SPEED_MULTIPLIER

#define START_OFFSET_MS			RAMP_UP_TIME_MS + RAMP_DOWN_TIME_MS + HIGH_DURATION_MS + LOW_DURATION_MS - OVERLAP
#define DELTA					MAX_BRIGHTNESS - MIN_BRIGHTNESS

enum mode_values { NOT_STARTED, RAMPING_UP, HIGH, RAMPING_DOWN, LOW};

// Set the initial conditions for all LEDs
uint16_t mode[3] = {NOT_STARTED, NOT_STARTED, NOT_STARTED};
uint64_t next_step[3];
uint16_t brightness[3] = {MIN_BRIGHTNESS, MIN_BRIGHTNESS, MIN_BRIGHTNESS};
uint64_t mode_timeout[3];

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

void start_fade(int led_id)
{
	if (mode[led_id] != NOT_STARTED)
	{
		DEBUG_BREAK
	}

	uint64_t start_time = get_time_in_ms();
	next_step[led_id] = RAMP_UP_TIME_MS / DELTA + start_time;
	mode_timeout[led_id] = start_time + RAMP_UP_TIME_MS;
	brightness[led_id] = MIN_BRIGHTNESS;
	TIMER_CompareBufSet(TIMER0, led_id, brightness[led_id]);
	mode[led_id] = RAMPING_UP;
}

void SysTick_Handler(void)
{
	uint64_t curr_time = get_time_in_ms();

	for(int i=0; i<3; i++)
	{
		switch (mode[i])
		{
		case NOT_STARTED:
			continue;
			break;
		case RAMPING_UP:
			if (next_step[i] <= curr_time)
			{
				next_step[i] = RAMP_UP_TIME_MS / DELTA + curr_time;
				TIMER_CompareBufSet(TIMER0, i, brightness[i]++);
			}
			if (mode_timeout[i] <= curr_time)
			{
				mode[i] = HIGH;
				mode_timeout[i] = curr_time + HIGH_DURATION_MS;
				brightness[i] = MAX_BRIGHTNESS;
				TIMER_CompareBufSet(TIMER0, i, brightness[i]);
			}
			break;
		case HIGH:
			if (mode_timeout[i] <= curr_time)
			{
				mode[i] = RAMPING_DOWN;
				TIMER_CompareBufSet(TIMER0, i, brightness[i]--);
				next_step[i] = RAMP_DOWN_TIME_MS / DELTA + curr_time;
				mode_timeout[i] = curr_time + RAMP_DOWN_TIME_MS;
			}
			break;
		case RAMPING_DOWN:
			if (next_step[i] <= curr_time)
			{
				next_step[i] = RAMP_DOWN_TIME_MS / DELTA + curr_time;
				TIMER_CompareBufSet(TIMER0, i, brightness[i]--);
			}
			if (mode_timeout[i] <= curr_time)
			{
				mode[i] = LOW;
				brightness[i] = MIN_BRIGHTNESS;
				TIMER_CompareBufSet(TIMER0, i, brightness[i]);
				mode_timeout[i] = curr_time + LOW_DURATION_MS;
			}
			break;
		case LOW:
			if (mode_timeout[i] <= curr_time)
			{
				mode[i] = NOT_STARTED;
			}
			break;
		}
	}
}

int main(void)
{
	CHIP_Init();

	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Set up TIMER0 for timekeeping
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_SECOND_TIMER_COUNT);

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0)

	// Enable LED output
	GPIO_PinModeSet(LED_PORT, LED_PIN0, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_PORT, LED_PIN1, gpioModePushPull, 0);
	GPIO_PinModeSet(LED_PORT, LED_PIN2, gpioModePushPull, 0);

	// Create the object initializer for LED PWM
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.cmoa = timerOutputActionToggle;

	// Configure TIMER0 CC channels
	TIMER_InitCC(TIMER0, 0, &timerCCInit);
	TIMER_InitCC(TIMER0, 1, &timerCCInit);
	TIMER_InitCC(TIMER0, 2, &timerCCInit);

	// Set up routes for PD1, PD2, PD3
	TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN
			| TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC3);

	// Set Top Value
	TIMER_TopSet(TIMER0, TIMER_TOP);

	// Set the PWM duty cycle here!
	TIMER_CompareBufSet(TIMER0, 0, 0);
	TIMER_CompareBufSet(TIMER0, 1, 0);
	TIMER_CompareBufSet(TIMER0, 2, 0);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit2 = TIMER_INIT_DEFAULT;
	timerInit2.prescale = timerPrescale256;

	TIMER_Init(TIMER0, &timerInit2);

	// Check for properly sized constants
	if ( DELTA == 0 || RAMP_UP_TIME_MS % DELTA || RAMP_DOWN_TIME_MS % DELTA)
	{
		DEBUG_BREAK
	}

	while (1)
	{
		// Sweep forward
		for (int i = 0; i < 3; i++)
		{
			start_fade(i);
			delay_ms(START_OFFSET_MS);
		}

		// This is the unusual step backward in the sweep
		start_fade(1);
		delay_ms(START_OFFSET_MS);
	}
}
