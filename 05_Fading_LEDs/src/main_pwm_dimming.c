
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#define LED_PORT	gpioPortE
#define LED_PIN		2

#define DEBUG_BREAK 			__asm__("BKPT #0");
#define TIMER_TOP				100
#define DUTY_CYCLE				1
#define TIMER_CHANNEL			2

int main(void)
{
	CHIP_Init();

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER3, true);

	// Enable LED output
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);

	// Create the timer count control object initializer
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	timerCCInit.cmoa = timerOutputActionToggle;

	// Configure CC channel 2
	TIMER_InitCC(TIMER3, TIMER_CHANNEL, &timerCCInit);

	// Route CC2 to location 1 (PE3) and enable pin for cc2
	TIMER3->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_LOCATION_LOC1);

	// Set Top Value
	TIMER_TopSet(TIMER3, TIMER_TOP);

	// Set the PWM duty cycle here!
	TIMER_CompareBufSet(TIMER3, TIMER_CHANNEL, DUTY_CYCLE);

	// Create a timerInit object, based on the API default
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale256;

	TIMER_Init(TIMER3, &timerInit);

	while (1)
		;
}

