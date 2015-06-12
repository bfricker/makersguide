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
#include "em_gpio.h"
#include "em_emu.h"
#include "segmentlcd.h"

#define LED_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3

#define BUTTON_PORT					gpioPortB
#define ADJUST_BUTTON_PIN			9
#define SET_BUTTON_PIN				10

#define DEBUG_BREAK 				__asm__("BKPT #0");
#define BLINK_HALF_PERIOD_MS		500

// Time segments
#define HOUR_SEGMENTS				0x60
#define MINUTE_SEGMENTS				0x18
enum segment_enum { NONE, TIME, HOURS, MINUTES };

// Storage for the time currently on the display
uint16_t display_hours = 0;
uint16_t display_minutes = 0;

// Blink state storage
uint16_t blink_segment = NONE;
uint32_t blink_timer = 0;
bool blink_state = 1;

// Button states
typedef struct button_struct_type
{
	bool short_press;
	bool long_press;
	uint32_t time_pressed;
	uint16_t pin;
} button_struct ;

button_struct set_button;
button_struct adjust_button;

// Third button is virtual, formed by pressing two together
button_struct program_button;

button_struct * button_array[3];

#define SET_BUTTON_INDEX  		0
#define ADJUST_BUTTON_INDEX 	1
#define PROGRAM_BUTTON_INDEX 	2

#define LONG_PRESS_THRESH		1000

volatile uint32_t msTicks = 0;

void delay(uint32_t milliseconds)
{
	uint32_t start = msTicks;
	while ((start + milliseconds) > msTicks)
		;
}

// Pass in the elapsed time before it times out, returns the future time
int32_t set_timeout_ms(int32_t timeout_ms)
{
	return msTicks + timeout_ms;
}

// Check to see if the future time has elapsed.
int32_t expired_ms(int32_t timeout_ms)
{
	if (timeout_ms < msTicks)
	{
		return true;
	}
	return false;
}

// Function used to update the display
// No bounds checking.
void display_time(const uint16_t set_hours, const uint16_t set_minutes)
{
	display_hours = set_hours;
	display_minutes = set_minutes;

	SegmentLCD_Number((display_hours << 8) + display_minutes);
}

// Blink time segments given by the segment_enum
// Only one time enum at a time
void blink(uint16_t segment)
{
	if (segment)
	{
		blink_timer = set_timeout_ms(BLINK_HALF_PERIOD_MS);
		blink_segment = segment;
	}
	else
	{
		blink_segment = NONE;
		display_time(display_hours, display_minutes);
	}
}

// Show the time segment given by segment_enum
// if show is false, hide it
void hide_segment(uint16_t segment)
{
	if (segment == TIME)
	{
		SegmentLCD_NumberOff();
	}
	else if (segment == HOURS)
	{
		for (int i=1; i<9; i++)
		{
			LCD_SegmentSetHigh(i, HOUR_SEGMENTS, 0);
		}
	}
	else if (segment == MINUTES)
	{
		for (int i=1; i<9; i++)
		{
			LCD_SegmentSetHigh(i, MINUTE_SEGMENTS, 0);
		}
	}
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	// Chip errata
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);

	// Set up the user interface buttons
	GPIO_PinModeSet(BUTTON_PORT, SET_BUTTON_PIN, gpioModeInput,  0);
	GPIO_PinModeSet(BUTTON_PORT, ADJUST_BUTTON_PIN, gpioModeInput,  0);

	set_button.short_press = false;
	set_button.long_press = false;
	set_button.pin = SET_BUTTON_PIN;

	adjust_button.short_press = false;
	adjust_button.long_press = false;
	adjust_button.pin = ADJUST_BUTTON_PIN;

	program_button.short_press = false;
	program_button.long_press = false;

	button_array[SET_BUTTON_INDEX] = &set_button;
	button_array[ADJUST_BUTTON_INDEX] = &adjust_button;
	button_array[PROGRAM_BUTTON_INDEX] = &program_button;

	// Set 1ms SysTick
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	// Enable LCD without voltage boost
	SegmentLCD_Init(false);

	// Turn on the colon in the time display
	SegmentLCD_Symbol(LCD_SYMBOL_COL10, 1);

	display_time(0,0);

	SegmentLCD_Write("OFF");

	while (1)
	{
		if (set_button.short_press)
		{
			if (!set_button.long_press)
			{
				GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull,  1);
			}
		}
		else
		{
			GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull,  0);
		}

		if (adjust_button.short_press)
		{
			if (!adjust_button.long_press)
			{
				GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull,  1);
			}
		}
		else
		{
			GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull,  0);
		}

		if (set_button.long_press)
		{
			GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull,  0);
		}

		if (adjust_button.long_press)
		{
			GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull,  0);
		}

		// This is a test of the buttons and will not program the timer yet
		if (program_button.long_press)
		{
			DEBUG_BREAK			// This is our test case
			delay(10);			// "Move to line" to here, release buttons
			if (program_button.long_press)
			{
				DEBUG_BREAK		// This should not occur
			}
		}
	}
}

void SysTick_Handler(void)
{
	msTicks++;

	// Handle blinking segments
	if (blink_segment)
	{
		if (expired_ms(blink_timer))
		{
			if (blink_state)
			{
				hide_segment(blink_segment);
			}
			else
			{
				display_time(display_hours, display_minutes);
			}
			blink_state = !blink_state;
			blink_timer = set_timeout_ms(BLINK_HALF_PERIOD_MS);
		}
	}

	// Keep track of button presses
	// High state is unpressed.  Low state is pressed
	// Only need to loop over the two physical buttons
	for (int i=0; i < 2; i++)
	{
		// A button high means it is unpressed, so clear things
		if (GPIO_PinInGet(BUTTON_PORT, button_array[i]->pin))
		{
			button_array[i]->short_press = false;
			button_array[i]->long_press = false;
			button_array[i]->time_pressed = 0;
		}
		else  // A button was found low, meaning pressed
		{
			// No need to do anything more if already found long_press
			if (!button_array[i]->long_press)
			{
				// Only need to fire short_press once
				if (!button_array[i]->time_pressed)
				{
					button_array[i]->short_press = true;
				}

				button_array[i]->time_pressed++;

				// Now check to see if we have a new long press
				if (button_array[i]->time_pressed > LONG_PRESS_THRESH)
				{
					button_array[i]->long_press = true;
				}
			}
		}
	}

	// Now take care of the virtual program button
	// Set a long press on the program button if other buttons are both long
	if (set_button.long_press && adjust_button.long_press)
	{
		program_button.long_press = true;
	}
	else
	{
		program_button.long_press = false;
	}
}
