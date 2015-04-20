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
			ITM_SendChar('\r');
			//buffer.head = fix_overflow(buffer.head);
		}

		ITM_SendChar(*string++);
		//buffer.data[buffer.head++] = *string++;
		//buffer.head = fix_overflow(buffer.head);
	}

//	// We need to kick off the  first transfer sometimes to get things going
//	INT_Disable();
//	if (!buffer.now_printing)
//	{
//		buffer.now_printing = true;
//		ITM_SendChar(buffer.data[buffer.tail++]);
//	}
//	INT_Enable();
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

void setupSWOForPrint(void)
{
  /* Enable GPIO clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
 
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
 
#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_LEOPARD_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;
 
  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) |GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
 
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;
 
  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));
 
  /* Enable trace in core debug */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR  = 0xC5ACCE55;
  ITM->TER  = 0x0;
  ITM->TCR  = 0x0;
  TPI->SPPR = 2;
  TPI->ACPR = 0xf;
  ITM->TPR  = 0x0;
  DWT->CTRL = 0x400003FE;
  ITM->TCR  = 0x0001000D;
  TPI->FFCR = 0x00000100;
  ITM->TER  = 0x1;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  enter_DefaultMode_from_RESET();

  for (volatile int i=0; i < 10000; i++)
	  ;

  print("\n");
  print("Hello World %d!", 2015);

  /* Infinite loop */
  while (1)
  {
//	  for (volatile int i=0; i < 10000; i++)
//		  ;
//	  print ("garbage ");
  }
}
