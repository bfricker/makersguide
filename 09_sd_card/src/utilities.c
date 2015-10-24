/*
 * utilities.c
 *
 * Holds commonly used functions
 */
#include <stdio.h>
#include <stdarg.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_int.h"
#include "utilities.h"

volatile uint32_t msTicks = 0;

#define PRINT_BUFFER_SIZE	1024
typedef struct print_buffer
{
	uint16_t head;
	uint16_t tail;
	uint8_t data[PRINT_BUFFER_SIZE];
	bool now_printing;
} print_buffer_struct;

print_buffer_struct buffer = { .head = 0, .tail = 0, .now_printing = false };

uint16_t fix_overflow(uint16_t index)
{
	if (index >= PRINT_BUFFER_SIZE)
	{
		return 0;
	}
	return index;
}

void send_string(char * string)
{
	while (*string != 0)
	{
		if (*string == '\n')
		{
			//USART_Tx(USART0, '\r');
			buffer.data[buffer.head++] = '\r';
			buffer.head = fix_overflow(buffer.head);
		}
		//USART_Tx(USART0, *string++);

		buffer.data[buffer.head++] = *string++;
		buffer.head = fix_overflow(buffer.head);
	}

	// We need to kick off the  first transfer sometimes to get things going
	INT_Disable();
	if (!buffer.now_printing)
	{
		buffer.now_printing = true;
		USART_Tx(USART0, buffer.data[buffer.tail++]);
		buffer.tail = fix_overflow(buffer.tail);
	}
	INT_Enable();
}

void print(const char* format, ...)
{
    char       msg[130];
    va_list    args;

    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args); // do check return value
    va_end(args);

    send_string(msg);
}

void delay(uint32_t milliseconds)
{
	uint32_t start = msTicks;
	while ((start + milliseconds) > msTicks)
		;
}

// Pass in the elapsed time before it times out, returns the future time
int32_t set_timeout_ms(int32_t timeout_ms)
{
	return msTicks + timeout_ms;
}

// Check to see if the future time has elapsed.
int32_t expired_ms(int32_t timeout_ms)
{
	if (timeout_ms < msTicks)
	{
		return true;
	}
	return false;
}

// If a button is low, that means it is pressed, so return true
bool get_button()
{
	if (GPIO_PinInGet(BUTTON_PORT, SET_BUTTON_PIN))
	{
		return false;
	}
	return true;
}

void set_led(int number, int level)
{
	if (number == 0)
	{
		GPIO_PinModeSet(LED_PORT, LED0_PIN, gpioModePushPull, level);
	}

	if (number == 1)
	{
		GPIO_PinModeSet(LED_PORT, LED1_PIN, gpioModePushPull, level);
	}
}

// Set up the USART0 clock before calling this function
void setup_utilities()
{
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	// Enable USART0 ints
	USART_IntClear(USART0, USART_IF_TXC);
	USART_IntEnable(USART0, USART_IF_TXC);
	NVIC_ClearPendingIRQ(USART0_TX_IRQn);
	NVIC_EnableIRQ(USART0_TX_IRQn);

	USART_IntClear(USART0, USART_IF_RXDATAV);
	USART_IntEnable(USART0, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);

	// Note: Must configure USART0 pins and clocks elsewhere!  BEFORE calling this function
}

void SysTick_Handler(void)
{
	msTicks++;
}

void USART0_TX_IRQHandler(void)
{
	if (USART0->IF & USART_IF_TXC)
	{
		// This flag is not automatically cleared like RXDATAV
		USART_IntClear(USART0, USART_IF_TXC);

		if (buffer.tail != buffer.head)
		{
			USART_Tx(USART0, buffer.data[buffer.tail++]);
			buffer.tail = fix_overflow(buffer.tail);
		}
		else
		{
			buffer.now_printing = false;
		}
	}
}
