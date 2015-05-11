
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "utilities.h"

#define TEST_JUMPER_PORT	gpioPortF
#define TEST_JUMPER_PIN		9

#define DEBOUNCE_TIME		300  // ms

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
	GPIO_PinModeSet(TEST_JUMPER_PORT, TEST_JUMPER_PIN, gpioModeInput,  0);

	bool pressed = false;
	int number_of_presses __attribute__((unused)) = 0;
	int debounce_pressed_timeout = 0;
	int debounce_released_timeout = 0;

	while (1)
	{
		if (GPIO_PinInGet(TEST_JUMPER_PORT, 9))
		{
			if (pressed == false && expired_ms(debounce_released_timeout))
			{
				// You could start some process here on the initial event
				// and it would be immediate
				number_of_presses++;
				pressed = true;
				debounce_pressed_timeout = set_timeout_ms(DEBOUNCE_TIME);
			}
		}
		else
		{
			if (pressed == true && expired_ms(debounce_pressed_timeout))
			{
				// You could start some process here on the release event
				// and it would be immediate
				pressed = false;
				debounce_released_timeout = set_timeout_ms(DEBOUNCE_TIME);
			}
		}
	}
}
