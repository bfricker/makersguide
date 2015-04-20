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
#include "em_rtc.h"
#include "em_msc.h"
#include "segmentlcd.h"

#define LED_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3

#define SOLENOID_PORT				gpioPortD
#define SOLENOID_PIN				1

#define BUTTON_PORT					gpioPortB
#define ADJUST_BUTTON_PIN			9
#define SET_BUTTON_PIN				10

#define DEBUG_BREAK 				__asm__("BKPT #0");
#define BLINK_HALF_PERIOD_MS		200

// Time segments
#define HOUR_SEGMENTS				0x60
#define MINUTE_SEGMENTS				0x18
enum segment_enum { NONE, TIME, HOURS, MINUTES };

// Storage for the time currently on the display
uint16_t display_hours = 0;
uint16_t display_minutes = 0;

// Storage for the program-to-program event memory
uint16_t start_hours = 12;
uint16_t start_minutes = 0;
uint16_t stop_hours = 12;
uint16_t stop_minutes = 0;

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

// All in ms
#define LONG_PRESS_THRESH		1000
#define BUTTON_DELAY			300
#define FASTER_BUTTON_DELAY		100

volatile uint32_t msTicks = 0;

// Control the timer from this global:
bool timer_on = false;

typedef struct rtc_struct_type
{
	uint32_t timer_start_seconds;
	uint32_t timer_stop_seconds;
} rtc_struct;

rtc_struct time_keeper;

#define END_OF_DAY				60*60*24

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

	SegmentLCD_Number(display_hours * 100 + display_minutes);
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

// Stores the hours/mins for clock time, start time, or stop time
void set_clock_time(int index, uint16_t hours, uint16_t minutes)
{
	// Set the time clock
	if (index == 0)
	{
		// Midnight is time zero
		RTC_CounterReset();
		RTC->CNT = hours * 3600 + minutes * 60;
	}
	else if (index == 1)
	{
		// Add 1 second so that RTC interrupt source is clear,
		// and not to confuse with the clock minute updates
		time_keeper.timer_start_seconds = hours * 3600 + minutes * 60 + 1;

		// Set up the RTC to trigger a start event
		RTC_CompareSet(0, time_keeper.timer_start_seconds);

		// Save for next program event
		start_hours = hours;
		start_minutes = minutes;
	}
	else if (index == 2)
	{
		// Add 1 second so that RTC interrupt source is clear,
		// and not to confuse with the clock minute updates
		time_keeper.timer_stop_seconds = hours * 3600 + minutes * 60 + 1;

		// Save for next program event
		stop_hours = hours;
		stop_minutes = minutes;
	}
}

uint16_t get_time(uint16_t segment)
{
	if (segment == HOURS)
	{
		return RTC->CNT / 3600;
	}
	else if (segment == MINUTES)
	{
		uint16_t leftover_seconds = RTC->CNT % 3600;
		return leftover_seconds / 60;
	}
	DEBUG_BREAK;
	return 0;
}

// Use buttons to set time, start, stop
// Stores those values to memory
void program_timer()
{
	static bool initial_programming = true;

	// Disable RTC interrupts while in here...
	RTC_IntDisable(0);
	RTC_IntDisable(1);

	delay(BUTTON_DELAY);

	for (int i=0; i < 3; i++)
	{
		if (i == 0)
		{
			SegmentLCD_Write("Clock");
		}
		else if (i == 1)
		{
			SegmentLCD_Write("Start");
			if (initial_programming)
			{
				start_hours = display_hours;
				start_minutes = display_minutes;
			}
			display_hours = start_hours;
			display_minutes = start_minutes;
		}
		else if (i == 2)
		{
			SegmentLCD_Write("Stop");
			if (initial_programming)
			{
				stop_hours = start_hours;
				stop_minutes = start_minutes;
			}
			display_hours = stop_hours;
			display_minutes = stop_minutes;
		}

		display_time(display_hours, display_minutes);
		blink(HOURS);
		delay(BUTTON_DELAY);

		// Set the hours
		while (!set_button.short_press)
		{
			if (adjust_button.short_press)
			{
				display_hours++;
				if (display_hours > 23)
				{
					display_hours = 1;
				}
				display_time(display_hours, display_minutes);
				// Delay so that we don't get double presses
				delay(BUTTON_DELAY);
			}
		}

		display_time(display_hours, display_minutes);
		blink(MINUTES);
		delay(BUTTON_DELAY);

		// Set the minutes
		while (!set_button.short_press)
		{
			if (adjust_button.short_press)
			{
				display_minutes++;
				if (display_minutes > 59)
				{
					display_minutes = 0;
				}
				display_time(display_hours, display_minutes);
				// Delay so that we don't get double presses
				delay(FASTER_BUTTON_DELAY);
			}
		}

		// Commit the clock hours and minutes to memory for the given index
		set_clock_time(i, display_hours, display_minutes);
	}

	display_hours = get_time(HOURS);
	display_minutes = get_time(MINUTES);
	blink(NONE);
	display_time(display_hours, display_minutes);
	// Delay so that we don't get double presses
	delay(BUTTON_DELAY);

	initial_programming = false;

	// Trigger interrupt every 60 seconds to update the time
	RTC_CompareSet(1, RTC->CNT + 60);

	// Now that programming is done, clear and enable interrupts
	RTC_IntClear(RTC_IFC_COMP0);
	RTC_IntClear(RTC_IFC_COMP1);
	RTC_IntEnable(RTC_IFC_COMP0);
	RTC_IntEnable(RTC_IFC_COMP1);
}

// This is the timekeeping clock
void setup_rtc()
{
	// Ensure LE modules are accessible
	CMU_ClockEnable(cmuClock_CORELE, true);

	// Enable LFACLK in CMU (will also enable oscillator if not enabled)
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

	// Use the prescaler to reduce power consumption. 2^15
	//CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32768);
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_512);

	//uint32_t rtcFreq = CMU_ClockFreqGet(cmuClock_RTC);

	// Enable clock to RTC module
	CMU_ClockEnable(cmuClock_RTC, true);

	// Set up the Real Time Clock as long-time timekeeping
	RTC_Init_TypeDef init = RTC_INIT_DEFAULT;
	init.comp0Top = false;
	RTC_Init(&init);

	// Enabling Interrupt from RTC
	NVIC_EnableIRQ(RTC_IRQn);
}

void setup_lcd()
{
	// Enable LCD without voltage boost
	SegmentLCD_Init(false);

	// Turn on the colon in the time display
	SegmentLCD_Symbol(LCD_SYMBOL_COL10, 1);

	display_time(0,0);

	SegmentLCD_Write("OFF");

	blink(TIME);
}

void setup_gpio_and_buttons()
{
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Make sure the soleniod is off at startup
	GPIO_PinModeSet(SOLENOID_PORT, SOLENOID_PIN, gpioModePushPull, 0);

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

	// Enable GPIO_ODD interrupt vector in NVIC
	//NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	// Configure interrupt on falling edge of SET_BUTTON_PIN
	GPIO_IntConfig(BUTTON_PORT, SET_BUTTON_PIN, false, true, true);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	// Chip errata
	CHIP_Init();

	setup_rtc();

	setup_gpio_and_buttons();

	setup_lcd();

	// Set 1ms SysTick
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	enum program_modes { INIT, PROGRAM, ON, OFF };

	uint16_t mode = INIT;
	uint16_t last_mode = INIT;

	while (1)
	{
		if (!set_button.short_press)
		{
			EMU_EnterEM2(false);
		}

		switch (mode)
		{
		case INIT:
			if (set_button.short_press || program_button.long_press)
			{
				mode = PROGRAM;
			}
			break;
		case PROGRAM:
			program_timer();
			last_mode = PROGRAM;
			mode = ON;
			break;
		case ON:
			if (mode != last_mode)
			{
				SegmentLCD_Write("ON");
				timer_on = true;
				last_mode = ON;
				RTC_CompareSet(0, time_keeper.timer_start_seconds);
				//SysTick->CTRL = 0;
			}
			if (program_button.long_press)
			{
				mode = PROGRAM;
			}
			else if (set_button.short_press)
			{
				mode = OFF;
				// Delay so that we don't get double presses
				delay(BUTTON_DELAY);
			}
			break;
		case OFF:
			if (mode != last_mode)
			{
				SegmentLCD_Write("OFF");
				timer_on = false;
				last_mode = OFF;
				//SysTick->CTRL = 0;
			}
			if (program_button.long_press)
			{
				mode = PROGRAM;
			}
			else if (set_button.short_press)
			{
				mode = ON;
				// Delay so that we don't get double presses
				delay(BUTTON_DELAY);
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

					// Stop firing short presses
					button_array[i]->short_press = false;
				}
			}
		}
	}

	// Now take care of the virtual program button
	// First clear any long presses that may be present
	if (program_button.long_press)
	{
		if (!set_button.long_press && !adjust_button.long_press)
		{
			program_button.long_press = false;
		}
	}

	// Now set a long press on the program button if other buttons are long
	if (set_button.long_press && adjust_button.long_press)
	{
		program_button.long_press = true;
	}
}

void GPIO_EVEN_IRQHandler(void)
{
	// clear the interrupt
	GPIO_IntClear(0xffff);

//	uint32_t foo = SysTick->CTRL;
//
//	if (SysTick->CTRL == 0)
//	{
//		// Set 1ms SysTick
//		if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
//		{
//			DEBUG_BREAK;
//		}
//	}
//
//	set_button.short_press = true;
}

void run_sprinkler()
{
	// Set test LED to indicate solenoid is ON
	GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull, 1);

	GPIO_PinModeSet(SOLENOID_PORT, SOLENOID_PIN, gpioModePushPull, 1);
}

void stop_sprinkler()
{
	// Clear test LED to indicate solenoid is OFF
	GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull, 0);

	GPIO_PinModeSet(SOLENOID_PORT, SOLENOID_PIN, gpioModePushPull, 0);
}

void RTC_IRQHandler(void)
{
	// Check to see which counter has triggered
	if (RTC->IF & RTC_IF_COMP0)
	{
		// Within 10 seconds allows us to speed up the clock for testing
		if ((RTC->CNT - time_keeper.timer_start_seconds) < 10)
		{
			run_sprinkler();
			RTC_CompareSet(0, time_keeper.timer_stop_seconds);
		}
		else
		{
			RTC_IntDisable(0);
			stop_sprinkler();
		}
		RTC_IntClear(RTC_IFC_COMP0);
	}
	else  // A minute update has occurred
	{
		if (RTC->CNT >= END_OF_DAY)
		{
			RTC->CNT = 0;
		}
		RTC_CompareSet(1, RTC->CNT + 60);
		RTC_IntClear(RTC_IFC_COMP1);
		display_hours = get_time(HOURS);
		display_minutes = get_time(MINUTES);
		display_time(display_hours, display_minutes);
	}
}


// The following functions can be used to write to user page of flash

#define USER_PAGE_ADDR		0x0FE00000

msc_Return_TypeDef Serialnum_write (uint32_t number)
{
	msc_Return_TypeDef ret;
	uint32_t *addr = (uint32_t *)USER_PAGE_ADDR;
	MSC_Init();
	ret = MSC_WriteWord(addr, &number, sizeof(number));
	MSC_Deinit();
	return ret;
}

uint64_t Serialnum_read (void) {
	uint64_t num;
	uint32_t *data = (uint32_t*) USER_PAGE_ADDR;
	num = *data;
	return num;
}

void Userpage_erase(void){
	uint32_t *addr = (uint32_t *)USER_PAGE_ADDR;
	MSC_ErasePage(addr);
}
