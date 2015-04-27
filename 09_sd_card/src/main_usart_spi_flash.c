#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "utilities.h"

#define JEDEC_ID_CMD	0x9F

void usart_setup()
{
	// Set up the necessary peripheral clocks
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	GPIO_DriveModeSet(gpioPortD, gpioDriveModeLow);

	// Enable the GPIO pins for the USART, starting with CS
	// This is to avoid clocking the flash chip when we set CLK high
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPullDrive, 1);		// CS
	GPIO_PinModeSet(gpioPortD, 0, gpioModePushPullDrive, 0);		// MOSI
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);		// MISO
	GPIO_PinModeSet(gpioPortD, 2, gpioModePushPullDrive, 1);		// CLK

	// Enable the GPIO pins for the misc signals, leave pulled high
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPullDrive, 1);		// WP#
	GPIO_PinModeSet(gpioPortD, 5, gpioModePushPullDrive, 1);		// HOLD#

	// Initialize and enable the USART
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.clockMode = usartClockMode3;
	init.msbf = true;

	USART_InitSync(USART1, &init);

	// Connect the USART signals to the GPIO peripheral
	USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN |
			USART_ROUTE_CLKPEN | USART_ROUTE_LOCATION_LOC1;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	usart_setup();

	setup_utilities();

	delay(100);

	uint8_t result[3];
	uint8_t index = 0;

	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPullDrive, 0);

	// Send the command, discard the first response
	USART_SpiTransfer(USART1, JEDEC_ID_CMD);

	// Now send garbage, but keep the results
	result[index++] = USART_SpiTransfer(USART1, 0);
	result[index++] = USART_SpiTransfer(USART1, 0);
	result[index++] = USART_SpiTransfer(USART1, 0);

	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPullDrive, 1);

	// Check the result for what is expected from the Spansion spec
	if (result[0] != 1 || result[1] != 0x40 || result[2] != 0x13)
	{
		DEBUG_BREAK
	}

	while (1)
		;
}
