
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "utilities.h"

#define DELAY_VALUE					100 // ms

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	// Chip errata
	CHIP_Init();

	setup_utilities();

	CMU_ClockEnable(cmuClock_GPIO, true);

	// Set up the user interface buttons
	GPIO_PinModeSet(BUTTON_PORT, SET_BUTTON_PIN, gpioModeInput,  0);

	while (1)
	{
		if (get_button())
		{
			set_led(0, 1);
			delay(DELAY_VALUE);
			set_led(1, 1);
		}
		else
		{
			set_led(0, 0);
			set_led(1, 0);
		}
	}
}
