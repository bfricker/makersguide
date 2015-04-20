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

#define LED_PORT	gpioPortD
#define LED_PIN	14

#define BUTTON_PORT	gpioPortB
#define BUTTON_PIN	9

#define EM4_PORT			gpioPortC
#define EM4_PIN				9
#define EM4_WAKEUP_ENABLE	0x04	// Must change when changing w/u pin

void blink_one_cycle(void)
{
	// Turn on the LED
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 1);

	// Add some delay
	for(volatile long i=0; i<100000; i++)
	  ;

	// Turn off the LED
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

	// Add some more delay
	for(volatile long i=0; i<100000; i++);
	  ;
}

void enter_em4(void)
{
	// Set PC9 as an input, used to wake the system
	GPIO_PinModeSet(EM4_PORT, EM4_PIN, gpioModeInput, 0);

	EMU_EM4Init_TypeDef em4_init = EMU_EM4INIT_DEFAULT;
	EMU_EM4Init(&em4_init);

	// Retain GPIO modes while in EM4, to wake it up with button press
	GPIO->CTRL = 1;
	GPIO->EM4WUEN = EM4_WAKEUP_ENABLE;
	GPIO->EM4WUPOL = 0;	// Low signal is button pushed state
	GPIO->CMD = 1; 		// EM4WUCLR = 1, to clear all previous events

	// Wait for the button to be released before we go to sleep
	// or else we will immediately wake back up again
	while (!GPIO_PinInGet(EM4_PORT, EM4_PIN))
		;

       EMU_EnterEM4();
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(BUTTON_PORT, BUTTON_PIN, gpioModeInput, 0);

  // The initial state of the button is high when not pushed
  bool past_button_state = 1;
  // Start out not blinking
  bool blinking = true;

  while (1)
  {
	  // Grab the state of the button, 1 for high voltage, 0 for low
	  bool live_button_state = GPIO_PinInGet(gpioPortB, 9);

	  // Invert the blinking mode every time a button is pressed
	  // which generates a low voltage on a pin
	  if (past_button_state == 1 && live_button_state == 0)
	  {
		  past_button_state = 0;

		  // Invert the blinking mode, so that it is buffered and will
		  // keep blinking/not blinking when the button is released
		  blinking = ~blinking;
	  }


	  // Reset the past state when the button is released
	  if (live_button_state == 1)
	  {
		  past_button_state = 1;
	  }

	  // Finally decide if there is going to be a blink cycle or not
	  if (blinking)
	  {
		  blink_one_cycle();
	  }
	  else
	  {
		  enter_em4();
	  }
  }
}

