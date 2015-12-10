// main.c
// Rebuilt solution for playing .wav files over I2S
// Chapter 13

#include "em_device.h"
#include "em_system.h"
#include "em_chip.h"
#include "em_cmu.h"

#include "i2s_helpers.h"
#include "utilities.h"

int main(void)
{
	CHIP_Init();

	/* Use 32MHZ HFXO as core clock frequency, need high speed for 44.1kHz stereo */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	/* Start clocks */
	CMU_ClockEnable(cmuClock_DMA, true);

	// Get the systick running for delay() functions
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	// This creates MCLK for the I2S chip from TIMER1
	create_gpio_clock();

	// Enable the PB1 pushbutton on the devkit
	setup_pushbutton();

	// Enable the I2S output pins
	I2S_init();

	// Give the I2S chip time to get started
	delay(100);

//	int track = 0;
	while (true)
	{
		int track = get_next_track();
		play_sound(track);

		// Debounce the switch...
		delay(750);

		while (!get_button())
			;
	}
}

// Define the systick for the delay function
extern uint32_t msTicks;
void SysTick_Handler(void)
{
	msTicks++;
}

//// We may want to use this someday with a DAC...
//// so keep these handy to replace I2S calls
//CMU_ClockEnable(cmuClock_DAC0, true);
//CMU_ClockEnable(cmuClock_TIMER0, true);
//CMU_ClockEnable(cmuClock_PRS, true);
//
//DAC_setup();
//
///* Start timer which will trig DMA ... */
//TIMER_setup();
