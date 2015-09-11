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
#include "InitDevice.h"
#include "em_i2c.h"
#include "utilities.h"

#define DEBUG_BREAK			__asm__("BKPT #0");

#define ADXL345_ADDRESS		0x53
#define TX_ARRAY_SIZE		10
#define RX_ARRAY_SIZE		10

// Globals for persistent storage
uint8_t tx_array[TX_ARRAY_SIZE];
uint8_t rx_array[RX_ARRAY_SIZE];

// Used by the read/write and register_read functions
void i2c_transfer(uint16_t device_addr, uint8_t tx_array[], uint8_t rx_array[], uint16_t tx_len, uint16_t rx_len, uint8_t flag)
{
	// Transfer structure
	I2C_TransferSeq_TypeDef i2cTransfer;

	// Initialize I2C transfer
	I2C_TransferReturn_TypeDef result;
	i2cTransfer.addr          = device_addr;
	i2cTransfer.flags         = flag;
	i2cTransfer.buf[0].data   = tx_array;
	i2cTransfer.buf[0].len    = tx_len;
	i2cTransfer.buf[1].data   = rx_array;
	i2cTransfer.buf[1].len    = rx_len;

	// Set up the transfer
	result = I2C_TransferInit(I2C0, &i2cTransfer);

	// Do it until the transfer is done
	while (result != i2cTransferDone)
	{
		if (result != i2cTransferInProgress)
		{
			DEBUG_BREAK;
		}
		result = I2C_Transfer(I2C0);
	}
}

void i2c_read_register(uint8_t reg_offset, uint8_t rx_array[], uint16_t rx_len)
{
	uint8_t tx_array[1] = {reg_offset};
	i2c_transfer((ADXL345_ADDRESS << 1), tx_array, rx_array, 2, rx_len, I2C_FLAG_WRITE_READ);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	enter_DefaultMode_from_RESET();

	setup_utilities();

	delay(100);

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	I2C_Init(I2C0, &i2cInit);

	i2c_read_register(0, rx_array, 5);

	/* Infinite loop */
	while (1) {
  }
}
