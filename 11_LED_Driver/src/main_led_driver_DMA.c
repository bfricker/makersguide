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
#include "em_usart.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "utilities.h"
#include "dmadrv.h"

#define CONTROL_PORT					gpioPortD
#define XLAT_PIN						1
#define VPRG_PIN						3
#define BLANK_PIN						4
#define SIN_PIN							0
#define SCLK_PIN						2
#define GSCLK_PIN						7

#define DOT_CORRECTION_MODE				0
#define GRAYSCALE_MODE					1
#define MAX_CHANNELS					16
#define MAX_GRAYSCALE					4095	// 4096 - 1 for zero
#define MAX_DOT_CORRECTION				63		// 64 - 1 for zero
#define MAX_STREAM_BIT_LEN				192
#define DC_STREAM_BIT_LEN				96		// DC stands for Dot Correction
#define DC_BIT_LEN						6
#define GS_BIT_LEN						12		// GS stands for Grayscale

#define DEBUG_BREAK 					__asm__("BKPT #0");

unsigned int dma_channel;

// Transfer Flag
volatile bool dma_in_progress;

// Structs for LED driver
typedef struct LED_bit_fields
{							// Number after colon is field width
	uint8_t dot_correction : DC_BIT_LEN;
	uint16_t grayscale: GS_BIT_LEN;
} LED_data_struct;

typedef struct LED_stream
{
	LED_data_struct channel[MAX_CHANNELS];
	bool mode;
} LED_stream_struct;

LED_stream_struct stream;

// This must be global to be used for DMA
uint8_t stream_buffer[MAX_STREAM_BIT_LEN/8];

typedef struct color_code_bits
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} color_code_struct;

#define WHITE   { 0xFF, 0xFF, 0xFF }
#define RED		{ 0xFF, 0x00, 0x00 }
#define GREEN	{ 0x00, 0xFF, 0x00 }
#define BLUE	{ 0x00, 0x00, 0xFF }

#define PURPLE 	{ 0xFF, 0x00, 0xFF }
#define YELLOW  { 0xFF, 0xFF, 0x00 }
#define ORANGE	{ 0xFF, 0x0F, 0x00 }

void dma_transfer_complete(unsigned int channel, bool primary, void *user)
{
	// Clear flag to indicate that transfer is complete
	dma_in_progress = false;
}

// Pack dot correction byte for the serial stream
uint8_t pack_dc_byte(uint8_t byte_position)
{
	const uint8_t starting_bit = byte_position * 8;
	const uint8_t dc_start_position = starting_bit / DC_BIT_LEN;
	const uint8_t dc_end_position = dc_start_position + 1;

	const uint8_t dc_lo = stream.channel[dc_start_position].dot_correction;
	const uint8_t dc_hi = stream.channel[dc_end_position].dot_correction;

	const uint8_t slot = byte_position % 3;

	switch (slot)
	{
	case 0:
		return dc_lo | (dc_hi << 6);
		break;
	case 1:
		return (dc_lo >> 2) | (dc_hi << 4);
		break;
	case 2:
		return (dc_lo >> 4) | (dc_hi << 2);
		break;
	}
	return 0;  // Will never occur, but makes compiler happy
}

// Pack grayscale byte for the serial stream
uint8_t pack_gs_byte(uint8_t byte_position)
{
	const uint8_t start_channel = (uint32_t) (byte_position * 8) / GS_BIT_LEN;

	const uint8_t slot = byte_position % 3;

	uint16_t hi, lo;

	switch (slot)
	{
	case 0:
		return stream.channel[start_channel].grayscale;
		break;
	case 1:
		hi = ((uint16_t) stream.channel[start_channel+1].grayscale) << 4;
		lo = ((uint16_t) stream.channel[start_channel].grayscale) >> 4;
		return hi | lo;
		break;
	case 2:
		return stream.channel[start_channel].grayscale >> 4;
		break;
	}
	return 0;  // Will never occur, but makes the compiler happy
}

void write_serial_stream()
{
	int length;

	// Must pack the bits in backwards for DMA driver
	if (stream.mode == DOT_CORRECTION_MODE)
	{
		length = DC_STREAM_BIT_LEN / 8;
		for (int i=0; i < length; i++)
		{
			stream_buffer[length-i-1] = pack_dc_byte(i);
		}
	}
	else
	{
		length = MAX_STREAM_BIT_LEN/8;
		for (int i=0; i < length; i++)
		{
			stream_buffer[length-i-1] = pack_gs_byte(i);
		}
	}

	// Set/clear the VPRG pin
	if (stream.mode == DOT_CORRECTION_MODE)
	{
		GPIO_PinOutSet(CONTROL_PORT, VPRG_PIN);
	}
	else
	{
		GPIO_PinOutClear(CONTROL_PORT, VPRG_PIN);
	}

	// Now write the stream
//	for (int i=length-1; i>=0; i--)
//	{
//		USART_Tx(USART1, stream_buffer[i]);
//	}
//
//	while (!(USART1->STATUS & USART_STATUS_TXC))
//		;

	dma_in_progress = true;

	// Start the DMA transfer.
	DMADRV_MemoryPeripheral( dma_channel,
						   dmadrvPeripheralSignal_USART1_TXBL,
						   (void*)&(USART1->TXDATA),
						   stream_buffer,
						   true,
						   length,
						   dmadrvDataSize1,
						   (void *) dma_transfer_complete,
						   NULL );

	while (dma_in_progress)
		; //EMU_EnterEM2(true);

	for (volatile int i=0; i < 10000; i++)
		;

	// Latch the data
	GPIO_PinOutSet(CONTROL_PORT, XLAT_PIN);
	for (volatile int i=0; i < 100; i++)
			;
	GPIO_PinOutClear(CONTROL_PORT, XLAT_PIN);
}

// Sets the color in memory but does not write it
void set_color_buffer(uint8_t led_number, color_code_struct color)
{
	const uint8_t ch0 = led_number * 3;
	const uint8_t ch1 = ch0 + 1;
	const uint8_t ch2 = ch1 + 1;

	// Shift the 8-bit RGB code over by 4 to get to 12 bits of GS
	stream.channel[ch0].grayscale = (uint16_t) (color.red << 4);
	stream.channel[ch1].grayscale = (uint16_t) (color.green << 4);
	stream.channel[ch2].grayscale = (uint16_t) (color.blue << 4);
}

void set_current_buffer(uint8_t led_number, color_code_struct color)
{
	const uint8_t ch0 = led_number * 3;
	const uint8_t ch1 = ch0 + 1;
	const uint8_t ch2 = ch1 + 1;

	stream.channel[ch0].dot_correction = color.red;
	stream.channel[ch1].dot_correction = color.green;
	stream.channel[ch2].dot_correction = color.blue;
}

void all_white()
{
	const color_code_struct white = WHITE;
	set_color_buffer(0, white);
	set_color_buffer(1, white);
	set_color_buffer(2, white);

	// Now send the stream to the TLC5940
	stream.mode = GRAYSCALE_MODE;			// Now write the GS data
	write_serial_stream();
}

void rgb_display()
{
	const color_code_struct red = RED;
	set_color_buffer(0, red);

	const color_code_struct green = GREEN;
	set_color_buffer(1, green);

	const color_code_struct blue = BLUE;
	set_color_buffer(2, blue);

	// Now send the stream to the TLC5940
	stream.mode = GRAYSCALE_MODE;			// Now write the GS data
	write_serial_stream();
}

void yellow_purple()
{
	const color_code_struct purple = PURPLE;
	set_color_buffer(0, purple);

	const color_code_struct yellow = YELLOW;
	set_color_buffer(1, yellow);

	set_color_buffer(2, purple);

	// Now send the stream to the TLC5940
	stream.mode = GRAYSCALE_MODE;			// Now write the GS data
	write_serial_stream();
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	/* Chip errata */
	CHIP_Init();

	// Turn on the peripheral clocks
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	Ecode_t result;

	// Initialize DMA.
	result = DMADRV_Init();
	if (result != ECODE_EMDRV_DMADRV_OK)
	{
		DEBUG_BREAK
	}

	// Request a DMA channel.
	result = DMADRV_AllocateChannel( &dma_channel, NULL );
	if (result != ECODE_EMDRV_DMADRV_OK)
	{
		DEBUG_BREAK
	}

	// Configure the USART peripheral
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.baudrate = 50000;
	init.msbf = true;		// MSB first, per the TLC5940 spec
	USART_InitSync(USART1, &init);

	// Route the peripheral to the GPIO block
	USART1->ROUTE = USART_ROUTE_TXPEN | 		// US1_TX
			        USART_ROUTE_CLKPEN | 		// US1_CLK
			        USART_ROUTE_LOCATION_LOC1;	// Location #1

	// Configure both the USART and control pins in GPIO
	GPIO_DriveModeSet(gpioPortD, gpioDriveModeLowest);
	GPIO_PinModeSet(CONTROL_PORT, SIN_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(CONTROL_PORT, SCLK_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(CONTROL_PORT, XLAT_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(CONTROL_PORT, VPRG_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(CONTROL_PORT, BLANK_PIN, gpioModePushPullDrive, 0);
	GPIO_PinModeSet(CONTROL_PORT, GSCLK_PIN, gpioModePushPullDrive, 0);

	// Start with DC Data
	stream.mode = DOT_CORRECTION_MODE;
	for (int i=0; i<MAX_CHANNELS; i++)
	{
//	      stream.channel[i].dot_correction = 0b100001;		// Test pattern
//	      stream.channel[i].grayscale = 0b100000000001;		// Test pattern
		stream.channel[i].dot_correction = 0xF; //MAX_DOT_CORRECTION;
		stream.channel[i].grayscale = 0; //MAX_GRAYSCALE;
	}

	// Write the DC Data
	write_serial_stream();

	//all_white();
	//rgb_display();
	yellow_purple();

	int count = 0;
	while (1)
	{
		if (count %2)
		{
			GPIO_PinOutSet(CONTROL_PORT, GSCLK_PIN);
		}
		else
		{
			GPIO_PinOutClear(CONTROL_PORT, GSCLK_PIN);
		}
		count++;

		if (count > 4096)
		{
			GPIO_PinOutSet(CONTROL_PORT, BLANK_PIN);
			for (volatile int i=0; i < 10; i++)
					;
			GPIO_PinOutClear(CONTROL_PORT, BLANK_PIN);
			count = 0;
		}

		for (volatile int i=0; i < 10; i++)
			;
	}
}
