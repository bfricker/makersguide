#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "utilities.h"
#include "spidrv.h"

// Commands
#define JEDEC_ID_CMD	0x9F
#define STATUS			0x05
#define WR_ENABLE		0x06
#define PAGE_PROGRAM	0x02
#define CHIP_ERASE		0xC7
#define READ_DATA		0x03

// Bit positions for status register
#define WIP_BIT			1 << 0
#define WEL_BIT			1 << 1

SPIDRV_HandleData_t handleData;
SPIDRV_Handle_t handle = &handleData;

void spidrv_setup()
{
	// Set up the necessary peripheral clocks
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_DriveModeSet(gpioPortD, gpioDriveModeLow);

	// Enable the GPIO pins for the misc signals, leave pulled high
	GPIO_PinModeSet(gpioPortD, 4, gpioModePushPullDrive, 1);		// WP#
	GPIO_PinModeSet(gpioPortD, 5, gpioModePushPullDrive, 1);		// HOLD#

	// Initialize and enable the SPIDRV
	SPIDRV_Init_t initData = SPIDRV_MASTER_USART1;
	initData.clockMode = spidrvClockMode3;

	// Initialize a SPI driver instance
	SPIDRV_Init( handle, &initData );
}

// Writes a single byte
void config_write(uint8_t command)
{
	const int size = 1;
	uint8_t result[size];
	uint8_t tx_data[size];

	tx_data[0] = command;

	SPIDRV_MTransferB( handle, &tx_data, &result, size);
}

// Reads the status
uint8_t read_status()
{
	const int size = 2;
	uint8_t result[size];
	uint8_t tx_data[size];

	tx_data[0] = STATUS;

	SPIDRV_MTransferB( handle, &tx_data, &result, size);
	return result[1];
}

void chip_erase()
{
	uint8_t status;
	config_write(WR_ENABLE);

	status = read_status();
	if (!(status & WEL_BIT))
	{
		DEBUG_BREAK
	}

	config_write(CHIP_ERASE);

	do status = read_status();
	while (status & WIP_BIT);
}

void read_memory(uint32_t address, uint8_t result[], uint32_t num_of_bytes)
{
	uint8_t tx_data[256+4];
	uint8_t rx_data[256+4];

	tx_data[0] = READ_DATA;
	tx_data[1] = (address >> 16);
	tx_data[2] = (address >> 8);
	tx_data[3] = address;

	SPIDRV_MTransferB( handle, &tx_data, &rx_data, num_of_bytes);

	// Fill the result from the right index
	for (int i=0; i< 256; i++)
	{
		result[i] = rx_data[i+4];
	}
}

void write_memory(uint32_t address, uint8_t data_buffer[], uint32_t num_of_bytes)
{
	if (num_of_bytes > 256) DEBUG_BREAK

	uint8_t status;
	uint8_t dummy_rx[256+4];
	uint8_t tx_data[256 + 4];  // Need room for cmd + three address bytes

	tx_data[0] = PAGE_PROGRAM;
	tx_data[1] = (address >> 16);
	tx_data[2] = (address >> 8);
	tx_data[3] = address;

	for (int i=0; i < 256; i++)
	{
		if (i >= num_of_bytes) break;
		tx_data[i+4] = data_buffer[i];
	}

	config_write(WR_ENABLE);
	SPIDRV_MTransferB( handle, &tx_data, &dummy_rx, num_of_bytes);

	do 	status = read_status();
	while (status & WIP_BIT);
}

//void TransferComplete( SPIDRV_Handle_t handle,
//                       Ecode_t transferStatus,
//                       int itemsTransferred )
//{
//  if ( transferStatus == ECODE_EMDRV_SPIDRV_OK )
//  {
//    // Success !
//  }
//}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	spidrv_setup();

	setup_utilities();

	delay(100);

	uint8_t result[256];
	uint8_t tx_data[256];

	tx_data[0] = JEDEC_ID_CMD;

	SPIDRV_MTransferB( handle, &tx_data, &result, 4);

	// Check the result for what is expected from the Spansion spec
	if (result[1] != 1 || result[2] != 0x40 || result[3] != 0x13)
	{
		DEBUG_BREAK
	}

	// Never enter here except with debugger and “Move to Line”
	if (result[1] == 0x2)
	{
		for (int i=0; i < 256; i++) tx_data[i] = i;

		chip_erase();
		read_memory(0, result, 256);
		write_memory(0, tx_data, 256);
		read_memory(0, result, 256);
	}

	read_memory(0, result, 256);

	while (1)
		;
}
