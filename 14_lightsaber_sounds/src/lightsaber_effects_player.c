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
#include <stdlib.h>
#include "adxl.h"
#include "utilities.h"

#define FREE_FALL_INT_PIN	12
#define ACTIVITY_INT_PIN	11

// This will get called whenever the GPIO interrupt occurs
// when we pass it into the set_gpio_interrupt function below
int GPIO_int_callback(uint8_t pin)
{
	bool pin_state = GPIO_PinInGet(gpioPortB, pin);

	// Set the appropriate LED based on the state of the INT1/INT2 pin
	if (pin == ACTIVITY_INT_PIN) set_led(0,pin_state);
	if (pin == FREE_FALL_INT_PIN) set_led(1,pin_state);

	return 0;
}

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

	i2c_setup();

	// Offset zero is Device ID
	uint16_t value = i2c_read_register(ADXL345_REG_DEVID);

	if (value != DEVICE_ID)
	{
		DEBUG_BREAK
	}

	i2c_init_registers();

	set_gpio_interrupt(gpioPortB,  ACTIVITY_INT_PIN, true, false, (GPIOINT_IrqCallbackPtr_t) GPIO_int_callback);
	set_gpio_interrupt(gpioPortB,  FREE_FALL_INT_PIN, true, false, (GPIOINT_IrqCallbackPtr_t) GPIO_int_callback);

	// We need to start the sound, which starts DMA, before we set up the DAC_TIMER
	play_sound(SABER_IDLE);
	DAC_setup();
	DAC_TIMER_setup();

	uint32_t adxl_debounce_timer = set_timeout_ms(500);

	while (1)
	{
		play_sound(SABER_IDLE);

		// Clear interrupts by reading the INT_SOURCE register
		uint8_t int_source = i2c_read_register(ADXL345_REG_INT_SOURCE);
		if (int_source & ADXL345_INT_Activity)
		{
			// Clear interrupts by reading the INT_SOURCE register
			i2c_read_register(ADXL345_REG_INT_SOURCE);

			if (expired_ms(adxl_debounce_timer))
			{
				add_track(SABER_SWING);
				adxl_debounce_timer = set_timeout_ms(500);
			}
		}
	}
}

// Define the systick for the delay function
extern uint32_t msTicks;
void SysTick_Handler(void)
{
	msTicks++;
}
