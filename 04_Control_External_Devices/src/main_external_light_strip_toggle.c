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

#define LED_PORT	gpioPortD
#define LED_PIN	14

#define BUTTON_PORT	gpioPortB
#define BUTTON_PIN	9

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
  bool blinking = false;

  while (1)
  {
	  // Grab the state of the button, 1 for high voltage, 0 for low
	  bool live_button_state = GPIO_PinInGet(BUTTON_PORT, BUTTON_PIN);

	  // Invert the blinking mode every time a button is pressed
	  // which generates a low voltage on a pin
	  if (past_button_state == 1 && live_button_state == 0)
	  {
		  past_button_state = 0;

		  // Invert the blinking mode, so that it is buffered and will
		  // keep blinking/not blinking when the button is released
		  blinking = !blinking;
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
    }
}
