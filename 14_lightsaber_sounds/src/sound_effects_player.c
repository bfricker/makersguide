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
#include "dac_helpers.h"
#include "utilities.h"

int main(void)
{
	CHIP_Init();

	// Need to boost the clock to get above 7kHz
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_DMA, true);

	// Enable GPIO output for Timer3, Compare Channel 2 on PE2
	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);

	// Show the sample rate on PE1 for debug
	GPIO_PinModeSet(gpioPortE, 1, gpioModePushPull, 0);

	// Get the systick running for delay() functions
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	int track = get_next_track();
	play_sound(track);
	DAC_setup();
	DAC_TIMER_setup();

	while (1) ;

//	// Enable the PB1 pushbutton on the devkit
//	setup_pushbutton();
//
//	while (true)
//	{
//		int track = get_next_track();
//		play_sound(track);
//
//		// Debounce the switch...
//		delay(750);
//
//		while (!get_button())
//			;
//	}
}

// Define the systick for the delay function
extern uint32_t msTicks;
void SysTick_Handler(void)
{
	msTicks++;
}
