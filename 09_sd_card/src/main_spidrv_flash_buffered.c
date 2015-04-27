#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_timer.h"
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

#define BUFFER_SIZE		256

uint8_t buffer_a[BUFFER_SIZE];
uint8_t buffer_b[BUFFER_SIZE];

bool programming_chip = false;
uint8_t * buffer_ptr = buffer_a;
int buffer_index = 0;

int flash_address = 0;

#define ONE_MS_TIMER_COUNT		14

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

void TransferComplete( SPIDRV_Handle_t handle,
                       Ecode_t transferStatus,
                       int itemsTransferred )
{
  if ( transferStatus == ECODE_EMDRV_SPIDRV_OK )
  {
    // Success !
  }
}

void write_memory(uint32_t address, uint8_t data_buffer[], uint32_t num_of_bytes)
{
	if (num_of_bytes > 256) DEBUG_BREAK

	if (read_status() & WIP_BIT) DEBUG_BREAK

	//uint8_t status;
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
	SPIDRV_MTransfer( handle, &tx_data, &dummy_rx, num_of_bytes, &TransferComplete);

	// Comment these out so that SPIDRV can work in background
	//do 	status = read_status();
	//while (status & WIP_BIT);
}

void setup_serial_port()
{
	// All important clock enablement...
	CMU_ClockEnable(cmuClock_USART0, true);

	// Initialize and enable the USART
	USART_InitAsync_TypeDef initasync = USART_INITASYNC_DEFAULT;
	USART_InitAsync(USART0, &initasync);

	// Enable the GPIO pins for USART0, location 5
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);		// TX
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);		// RX

	USART0->ROUTE = USART_ROUTE_LOCATION_LOC5 | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

	USART_IntClear(USART0, USART_IF_RXDATAV);
	USART_IntEnable(USART0, USART_IF_RXDATAV);

	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
}

// Get counter started but no ints yet
void setup_timer()
{
	CMU_ClockEnable(cmuClock_TIMER1, true);

	// Set up TIMER1 for timekeeping
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale1024;

	TIMER_Init(TIMER1, &timerInit);

	// Wait for the timer to get going
	while (TIMER1->CNT == 0)
		;
}

// Disabled automatically when expires
void enable_timer_ints()
{
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

	// Reset the count
	TIMER1->CNT = 0;

	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_MS_TIMER_COUNT);
}

void start_flash_transfer()
{
	uint8_t * program_buffer = buffer_a;

	if (programming_chip)
	{
		if (buffer_ptr == buffer_a)
		{
			program_buffer = buffer_b;
		}
	}
	else if (buffer_ptr == buffer_b)
	{
		program_buffer = buffer_b;
	}

	write_memory(flash_address, program_buffer, BUFFER_SIZE);
	flash_address += BUFFER_SIZE;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	spidrv_setup();

	setup_utilities();

	setup_serial_port();

	setup_timer();

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

void USART0_RX_IRQHandler(void)
{
	if (USART0->IF & USART_IF_RXDATAV)
	{
		char test_char = USART_Rx(USART0);
		if (!programming_chip && test_char == 'p')
		{
			programming_chip = true;
			print("Erasing chip...\n");
			//chip_erase();
			print("Done.  Transfer file now.\n");

			enable_timer_ints();
			flash_address = 0;
		}
		else if (!programming_chip)
		{
			USART_Tx(USART0, test_char);
		}
		else if (programming_chip)
		{
			// Reset the timer count
			TIMER1->CNT = 0;

			buffer_ptr[buffer_index++] = test_char;
			if (buffer_index >= BUFFER_SIZE)
			{
				if (buffer_ptr == buffer_a)
				{
					buffer_ptr = buffer_b;
				}
				else
				{
					buffer_ptr = buffer_a;
				}
				buffer_index = 0;
			}

			start_flash_transfer();
		}
	}
}

// Called after last chuck of data is received
void TIMER1_IRQHandler(void)
{
	programming_chip = false;
	start_flash_transfer();
	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
	NVIC_DisableIRQ(TIMER1_IRQn);
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
}
