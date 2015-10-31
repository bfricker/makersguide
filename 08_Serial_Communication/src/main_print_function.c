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
#include <stdio.h>
#include <stdarg.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_int.h"
#include "InitDevice.h"

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

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  enter_DefaultMode_from_RESET();

  USART_IntClear(USART0, USART_IF_RXDATAV);
  USART_IntEnable(USART0, USART_IF_RXDATAV);

  USART_IntClear(USART0, USART_IF_TXC);
  USART_IntEnable(USART0, USART_IF_TXC);

  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);

  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
  NVIC_EnableIRQ(USART0_TX_IRQn);

  for (volatile int i=0; i < 10000; i++)
	  ;

  print("\n");
  //print("Hello World %d!", 2015);
#define HELLO_WORLD_STRING "Hello World %d!"
  print(HELLO_WORLD_STRING, 2015);

  /* Infinite loop */
  while (1)
  {
//	  for (volatile int i=0; i < 10000; i++)
//		  ;
//	  print ("garbage ");
  }
}

void USART0_RX_IRQHandler(void)
{
	if (USART0->IF & USART_IF_RXDATAV)
	{
		char test_char = USART_Rx(USART0);
		USART_Tx(USART0, test_char);
	}
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
