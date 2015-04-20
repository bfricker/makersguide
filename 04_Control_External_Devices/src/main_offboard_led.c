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

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  CMU_ClockEnable(cmuClock_GPIO, true);

  while (1)
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
}
