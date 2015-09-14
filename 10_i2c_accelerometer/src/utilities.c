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
#include "gpiointerrupt.h"

volatile uint32_t msTicks = 0;
USART_TypeDef *utility_usart;
bool gpio_interrupts_configured = false;

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
		USART_Tx(utility_usart, buffer.data[buffer.tail++]);
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


void setup_utilities(USART_TypeDef *usart)
{
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
	{
		DEBUG_BREAK;
	}

	utility_usart = usart;

	// Enable USART0 ints
	USART_IntClear(utility_usart, USART_IF_TXC);
	USART_IntEnable(utility_usart, USART_IF_TXC);

	IRQn_Type irq;
	if (utility_usart == USART0)
	{
		irq = USART0_TX_IRQn;
	}
	else if (utility_usart == USART1)
	{
		irq = USART1_TX_IRQn;
	}
	else
	{
		DEBUG_BREAK  // USART out of range
	}
	NVIC_ClearPendingIRQ(irq);
	NVIC_EnableIRQ(irq);

	// Note: Must configure USART0 pins and clocks elsewhere!
}

void SysTick_Handler(void)
{
	msTicks++;
}


void common_usart_handler()
{
	if (utility_usart->IF & USART_IF_TXC)
	// This flag is not automatically cleared like RXDATAV
	USART_IntClear(utility_usart, USART_IF_TXC);

	if (buffer.tail != buffer.head)
	{
		USART_Tx(utility_usart, buffer.data[buffer.tail++]);
		buffer.tail = fix_overflow(buffer.tail);
	}
	else
	{
		buffer.now_printing = false;
	}
}

void USART0_TX_IRQHandler(void)
{
	common_usart_handler();
}

void USART1_TX_IRQHandler(void)
{
	common_usart_handler();
}

// Enables the GPIO pin and sets up interrupts
// Sends control back to your own function in callbackPtr when it occurs
// Function prototype is like this: int GPIO_int_callback(uint8_t pin)
// Interrupts are automatically cleared before the callback is returned
void set_gpio_interrupt(uint8_t port, uint8_t pin, bool rising_edge, bool falling_edge, GPIOINT_IrqCallbackPtr_t callbackPtr)
{
	if (!gpio_interrupts_configured)
	{
		GPIOINT_Init();
		gpio_interrupts_configured = true;
	}

	// Enable the GPIO and filter with the 1 on DOUT
	GPIO_PinModeSet(port, pin, gpioModeInput, 1);

	// Register the callback function
	GPIOINT_CallbackRegister(pin, callbackPtr);

	//(port, pin, risingEdge, fallingEdge, enable)
	GPIO_IntConfig(port, pin, rising_edge, falling_edge, true);
	GPIO_IntEnable( 1<<pin);
}
