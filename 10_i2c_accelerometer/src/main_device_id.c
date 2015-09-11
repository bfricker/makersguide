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

#define ADXL345_ADDRESS		0x53 << 1
#define DEVICE_ID			0xE5
#define CMD_ARRAY_SIZE		1
#define DATA_ARRAY_SIZE		10

// Globals for persistent storage
uint8_t cmd_array[CMD_ARRAY_SIZE];
uint8_t data_array[DATA_ARRAY_SIZE];

// Used by the read_register and write_register functions
// data_array is read data for WRITE_READ and tx2 data for WRITE_WRITE
void i2c_transfer(uint16_t device_addr, uint8_t cmd_array[], uint8_t data_array[], uint16_t cmd_len, uint16_t data_len, uint8_t flag)
{
	// Transfer structure
	I2C_TransferSeq_TypeDef i2cTransfer;

	// Initialize I2C transfer
	I2C_TransferReturn_TypeDef result;
	i2cTransfer.addr          = device_addr;
	i2cTransfer.flags         = flag;
	i2cTransfer.buf[0].data   = cmd_array;
	i2cTransfer.buf[0].len    = cmd_len;

	// Note that WRITE_WRITE this is tx2 data
	i2cTransfer.buf[1].data   = data_array;
	i2cTransfer.buf[1].len    = data_len;

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

// Read a config register on an I2C device
// Tailored for the ADX345 device only i.e. 1 byte of TX
uint8_t i2c_read_register(uint8_t reg_offset)
{
	cmd_array[0] = reg_offset;
	i2c_transfer(ADXL345_ADDRESS, cmd_array, data_array, 1, 1, I2C_FLAG_WRITE_READ);
	return data_array[0];
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

	// Offset zero is Device ID
	uint16_t value = i2c_read_register(0);

	// Set an LED on the Starter Kit if success
	if (value == DEVICE_ID)
	{
		set_led(1,1);
	}

	// Infinite loop
	while (1) {
  }
}
