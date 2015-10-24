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
#define FLASH_STATUS	0x05
#define WR_ENABLE		0x06
#define PAGE_PROGRAM	0x02
#define CHIP_ERASE		0xC7
#define READ_DATA		0x03

// Bit positions for status register
#define WIP_BIT			1 << 0
#define WEL_BIT			1 << 1

SPIDRV_HandleData_t handleData;
SPIDRV_Handle_t handle = &handleData;

#define PAGE_SIZE			256
#define RING_BUFFER_SIZE	1024
#define SPI_TRANSFER_SIZE 	PAGE_SIZE + 4  // Account for SPI header

// Ring buffer indicies and tail pointer
uint8_t ring_buffer[RING_BUFFER_SIZE];
int ring_head_index = 0;
int ring_tail_index = 0;

// Flags that control logic flow
bool programming_chip = false;
bool start_flash_transfer = false;
bool perform_chip_erase = false;
bool spidrv_active = false;
bool timer_initialized = false;
bool finish_last_chunk = false;

// Counters
int flash_address = 0;
uint32_t total_bytes = 0;
uint32_t bytes_to_dump = 0;

extern uint32_t msTicks;
uint32_t start_ticks;

#define ONE_MS_TIMER_COUNT		14

void TransferComplete( SPIDRV_Handle_t handle,
                       Ecode_t transferStatus,
                       int itemsTransferred )
{
  if ( transferStatus == ECODE_EMDRV_SPIDRV_OK )
  {
    // Success !
	spidrv_active = false;
  }
}

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
	initData.bitRate = 5000000;
	initData.clockMode = spidrvClockMode3;

	// Initialize a SPI driver instance
	SPIDRV_Init( handle, &initData );
}

// Writes a single byte
void config_write(uint8_t command)
{
	while (spidrv_active)
		;

	const int size = 1;
	uint8_t result[size];
	uint8_t tx_data[size];

	tx_data[0] = command;

	SPIDRV_MTransferB( handle, &tx_data, &result, size);
}

// Reads the status
uint8_t read_status()
{
	while (spidrv_active)
		;

	const int size = 2;
	uint8_t result[size];
	uint8_t tx_data[size];

	tx_data[0] = FLASH_STATUS;

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
	uint8_t tx_data[SPI_TRANSFER_SIZE];
	uint8_t rx_data[SPI_TRANSFER_SIZE];

	tx_data[0] = READ_DATA;
	tx_data[1] = (address >> 16);
	tx_data[2] = (address >> 8);
	tx_data[3] = address;

	SPIDRV_MTransferB( handle, &tx_data, &rx_data, SPI_TRANSFER_SIZE);

	// Fill the result from the correct index
	for (int i=0; i< PAGE_SIZE; i++)
	{
		result[i] = rx_data[i+4];
	}
}

void write_memory(uint32_t address, uint8_t data_buffer[], uint32_t num_of_bytes)
{
	while (spidrv_active)
		;

	if (num_of_bytes > PAGE_SIZE) DEBUG_BREAK

	if (read_status() & WIP_BIT) DEBUG_BREAK

	//uint8_t status;
	uint8_t dummy_rx[SPI_TRANSFER_SIZE];
	uint8_t tx_data[SPI_TRANSFER_SIZE];  // Need room for cmd + three address bytes

	tx_data[0] = PAGE_PROGRAM;
	tx_data[1] = (address >> 16);
	tx_data[2] = (address >> 8);
	tx_data[3] = address;

	for (int i=0; i < PAGE_SIZE; i++)
	{
		if (i >= num_of_bytes) break;
		tx_data[i+4] = data_buffer[i];
	}

	config_write(WR_ENABLE);
	spidrv_active = true;
	SPIDRV_MTransfer( handle, &tx_data, &dummy_rx, SPI_TRANSFER_SIZE, TransferComplete);

	// Comment these out so that SPIDRV can work in background
	//do 	status = read_status();
	//while (status & WIP_BIT);
}

void configure_serial_port()
{
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	// All important clock enablement...
	CMU_ClockEnable(cmuClock_USART0, true);

	// Initialize and enable the USART
	USART_InitAsync_TypeDef initasync = USART_INITASYNC_DEFAULT;
	USART_InitAsync(USART0, &initasync);

	// Enable the GPIO pins for USART0, location 5
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);		// TX
	GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);		// RX

	USART0->ROUTE = USART_ROUTE_LOCATION_LOC5 | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;
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
	// Set TIMER Top value
	TIMER_TopSet(TIMER1, ONE_MS_TIMER_COUNT * 200);

	// Reset the count
	TIMER1->CNT = 0;

	TIMER_IntClear(TIMER1, TIMER_IF_OF);

	TIMER_IntEnable(TIMER1, TIMER_IF_OF);

	// Enable TIMER0 interrupt vector in NVIC
	NVIC_EnableIRQ(TIMER1_IRQn);

}

// Must always use this for putting data into
// ring buffer or else could exceed RING_BUFFER_SIZE
// This gets filled by the LEUART handler
void push_onto_ring_buffer(uint8_t byte)
{
	ring_buffer[ring_head_index++] = byte;
	if (ring_head_index >= RING_BUFFER_SIZE)
	{
		ring_head_index = 0;
	}
}

// Returns true if there is a PAGE_SIZE of data
// available, else false
bool ring_buffer_valid()
{
	int width = ring_head_index - ring_tail_index;
	if (width >= PAGE_SIZE || width < 0)
	{
		return true;
	}
	return false;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	spidrv_setup();

	configure_serial_port();	// This configures the clock used by setup_utilities()

	setup_utilities();

	setup_timer();

	delay(100);

	print("\nPress p to erase chip and start programming: \n");

	uint8_t result[PAGE_SIZE];
	uint8_t tx_data[PAGE_SIZE];

	tx_data[0] = JEDEC_ID_CMD;

	SPIDRV_MTransferB( handle, &tx_data, &result, 4);

	// Check the result for what is expected from the Spansion spec
	if (result[1] != 1 || result[2] != 0x40 || result[3] != 0x13)
	{
		DEBUG_BREAK
	}

	// Uncomment this line to read from the flash on boot up
	//bytes_to_dump = 628;

	enable_timer_ints();

	while (true)
	{
		if (!timer_initialized)
		{
			// Reset the count until the USART handler does this
			TIMER1->CNT = 0;
		}

		// Check to see if there is enough data to write to flash
		if (ring_buffer_valid())
		{
			write_memory(flash_address, &ring_buffer[ring_tail_index], PAGE_SIZE);
			flash_address += PAGE_SIZE;

			// Increment the tail index, and handle rollover
			ring_tail_index += PAGE_SIZE;
			if (ring_tail_index >= RING_BUFFER_SIZE)
			{
				ring_tail_index = 0;
			}
		}

		if (perform_chip_erase)
		{
			print("Erasing chip...\n");
			chip_erase();
			perform_chip_erase = false;
			print("Done.  Transfer file now.\n");
			flash_address = 0;
			programming_chip = true;
		}

		if (finish_last_chunk)
		{
			// Wait for any pending transfers to finish
			while (spidrv_active)
				;

			// Check to see if there is something left to transfer
			if (ring_tail_index != ring_head_index)
			{
				write_memory(flash_address, &ring_buffer[ring_tail_index], PAGE_SIZE);
			}

			// Let the flash write finish before starting the print function
			while (spidrv_active)
				;

			print("Transfer complete of %d bytes.\n", total_bytes);
			bytes_to_dump = total_bytes;
			finish_last_chunk = false;
		}

		if (bytes_to_dump > 0)
		{
			uint8_t read_chars[PAGE_SIZE];

			print("****  Data read from flash follows **** \n");
			delay(500);
			int i = 0;
			while (bytes_to_dump > 0)
			{
				read_memory(i, read_chars, PAGE_SIZE);
				for (int j=0; j < PAGE_SIZE; j++)
				{
					USART_Tx(USART0, read_chars[j]);
					if (read_chars[j] == '\r')
					{
						USART_Tx(USART0, '\n');
					}

					bytes_to_dump--;
					if (bytes_to_dump == 0)
					{
						break;
					}
				}
				i += PAGE_SIZE;
			}
		}
	}
}

void USART0_RX_IRQHandler(void)
{
	if (USART0->IF & USART_IF_RXDATAV)
	{
		char test_char = USART_Rx(USART0);
		if (!programming_chip && test_char == 'p')
		{
			perform_chip_erase = true;
		}
		else if (!programming_chip)
		{
			USART_Tx(USART0, test_char);
		}
		else
		{
			// Reset the timer count
			TIMER1->CNT = 0;

			// Tell the main loop to stop resetting TIMER->CNT
			timer_initialized = true;

			push_onto_ring_buffer(test_char);

			total_bytes++;
		}
	}
}

// Called after last chunk of data is received
void TIMER1_IRQHandler(void)
{
	if (!programming_chip)
	{
		TIMER1->CNT = 0;
		TIMER_IntClear(TIMER1, TIMER_IF_OF);
		return;
	}
	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
	NVIC_DisableIRQ(TIMER1_IRQn);
	TIMER_IntClear(TIMER1, TIMER_IF_OF);

	finish_last_chunk = true;
}
