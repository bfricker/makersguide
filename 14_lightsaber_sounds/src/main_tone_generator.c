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
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#define PWM_TIMER_CHANNEL		2
#define PWM_TIMER				TIMER3
#define SAMPLE_TIMER			TIMER1
#define SAMPLE_TIMER_INT		TIMER1_IRQn
#define TIMER_PRESCALE			timerPrescale2
#define TONE_FREQ				3500

// Storage for the max frequency available to this program
uint32_t max_frequency;

uint32_t top_value(uint32_t frequency)
{
	if (frequency == 0) return 0;

	return (100*max_frequency/frequency);
}

// This is the timer that controls the PWM rate and PWM value per sample
// This timer is routed to a GPIO and needs no interrupt to set the PWM value
// when the compare register is reached in the timer count
void setup_pwm_timer3()
{
	// Create the timer count control object initializer
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.cmoa = timerOutputActionClear;
	timerCCInit.cofoa = timerOutputActionSet;

	// Configure Compare Channel 2
	TIMER_InitCC(PWM_TIMER, PWM_TIMER_CHANNEL, &timerCCInit);

	// Route CC2 to location 1 (PE3) and enable pin for cc2
	PWM_TIMER->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC1);

	// Create a timerInit object, and set the freq to maximum
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = TIMER_PRESCALE;

	// Set Top Value
	TIMER_TopSet(PWM_TIMER, top_value(TONE_FREQ) / 10);

	TIMER_Init(PWM_TIMER, &timerInit);
}

// This is the timer that used to know when to fetch a sample to process
// This timer is only used to generate interrupts
void setup_sample_rate_timer1()
{
	// Create a timerInit object, and set the freq to maximum
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = TIMER_PRESCALE;

	// Set Top Value
	TIMER_TopSet(SAMPLE_TIMER, top_value(TONE_FREQ) );

	TIMER_Init(SAMPLE_TIMER, &timerInit);

	TIMER_IntEnable(SAMPLE_TIMER, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(SAMPLE_TIMER_INT);
}


uint32_t sine_wave_generator()
{
	const uint8_t lookup_table[10] = {5,8,10,10,8,5,2,0,0,2};
	static uint8_t count = 0;

	// Lookup the value
	uint32_t result = lookup_table[count];

	// Adjust count for next time and correct for overflow
	count++; if (count >= 10) count = 0;

	return result * top_value(TONE_FREQ) / 100;
}

int main(void)
{
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_TIMER3, true);

	// Need to boost the clock to get above 7kHz
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	// Calculate the max frequency supported by this program
	uint32_t timer_freq = CMU_ClockFreqGet(cmuClock_TIMER3);
	max_frequency = (timer_freq / (1 << TIMER_PRESCALE)) / 1000;

	// Set up our timers that do almost all the work
	setup_sample_rate_timer1();
	setup_pwm_timer3();

	// Enable GPIO output for Timer3, Compare Channel 2 on PE2
	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);

	// Show the sample rate on PE1 for debug
	GPIO_PinModeSet(gpioPortE, 1, gpioModePushPull, 0);

	while (1)
	{
	}
}

void TIMER1_IRQHandler(void)
{
	// Clear the interrupt
	TIMER_IntClear(SAMPLE_TIMER, TIMER_IF_OF);

	// This is for debug viewing on a scope
	GPIO_PinOutToggle(gpioPortE, 1);

	// Get the next sine value
	uint32_t pwm_duty = sine_wave_generator();

	// Set the new duty cycle for this sample based on the generator
	TIMER_CompareBufSet(PWM_TIMER, PWM_TIMER_CHANNEL, pwm_duty);
}
