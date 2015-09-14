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
#include "adxl.h"
#include "em_emu.h"
#include "em_gpio.h"

#define DEVICE_ID			0xE5
#define CMD_ARRAY_SIZE		1
#define DATA_ARRAY_SIZE		10
#define FREE_FALL_INT_PIN	12
#define ACTIVITY_INT_PIN	11

// Globals for persistent storage
uint8_t cmd_array[CMD_ARRAY_SIZE];
uint8_t data_array[DATA_ARRAY_SIZE];

typedef struct accel_sample_data
{
	int16_t x;
	int16_t y;
	int16_t z;
} accel_sample_struct;

accel_sample_struct sample;

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

// Write a config register on an I2C device
// Tailored for the ADX345 device only i.e. 1 byte of TX
void i2c_write_register(uint8_t reg_offset, uint8_t write_data)
{
	cmd_array[0] = reg_offset;
	data_array[0] = write_data;
	i2c_transfer(ADXL345_ADDRESS, cmd_array, data_array, 1, 1, I2C_FLAG_WRITE_WRITE);
}

void ADXL_init(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	I2C_Init(I2C0, &i2cInit);

	uint16_t value = i2c_read_register(ADXL345_REG_DEVID);

	// Set an LED on the Starter Kit if success
	if (value == DEVICE_ID)
	{
		set_led(1,1);
	}

	// Set range to 16g and FULL resolution
	i2c_write_register(ADXL345_REG_DATA_FORMAT, ADXL345_RANGE_16_G );

	// ADXL_BW_RATE = 50 hz, limited by I2C speed
	i2c_write_register(ADXL345_REG_BW_RATE, ADXL345_DATARATE_50_HZ);

	// Set up threshold for activity, found through trial and error
	i2c_write_register(ADXL345_REG_THRESH_ACT, 0x90);

	// Turn on the axes that will participate
	i2c_write_register(ADXL345_REG_ACT_INACT_CTL, ADXL345_ACT_ac_dc | ADXL345_ACT_X | ADXL345_ACT_Y | ADXL345_ACT_Z);

	// Set up free fall thresh
	i2c_write_register(ADXL345_REG_THRESH_FF, 0x6);

	// Set up free fall time
	i2c_write_register(ADXL345_REG_TIME_FF, 0x20);

	// Set up interrupt outputs
	i2c_write_register(ADXL345_REG_INT_ENABLE, ADXL345_INT_FREE_FALL | ADXL345_INT_Activity );
	//i2c_write_register(ADXL345_REG_INT_ENABLE, ADXL345_INT_FREE_FALL );

	// Route ADXL345_INT_Activity to INT1 and ADXL345_INT_FREE_FALL to INT2
	i2c_write_register(ADXL345_REG_INT_MAP, ADXL345_INT_FREE_FALL);

	// Clear interrupts by reading the INT_SOURCE register
	i2c_read_register(ADXL345_REG_INT_SOURCE);

	// Start measurement
	i2c_write_register(ADXL345_REG_POWER_CTL, 0x08);
}

accel_sample_struct read_accel_data(void)
{
	// Measurement data starts at DATAX0, and ends at DATAZ1, 6 bytes long
	cmd_array[0] = ADXL345_REG_DATAX0;

	// Read 6 bytes at once
	i2c_transfer(ADXL345_ADDRESS, cmd_array, data_array, 1, 6, I2C_FLAG_WRITE_READ);

	// Now pack the return structure with meaningful data
	sample.x = (data_array[1] << 8) | data_array[0];
	sample.y = (data_array[3] << 8) | data_array[2];
	sample.z = (data_array[5] << 8) | data_array[4];

	return sample;
}

// This will get called whenever the GPIO interrupt occurs
// when we pass it into the set_gpio_interrupt function below
int GPIO_int_callback(uint8_t pin)
{
	bool pin_state = GPIO_PinInGet(gpioPortB, pin);

	// Set the appropriate LED based on the state of the INT1/INT2 pin
	if (pin == ACTIVITY_INT_PIN) set_led(0,pin_state);
	if (pin == FREE_FALL_INT_PIN) set_led(1,pin_state);

	return 0;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
	CHIP_Init();

	enter_DefaultMode_from_RESET();

	setup_utilities(USART1);

	delay(100);

	ADXL_init();

	set_gpio_interrupt(gpioPortB,  ACTIVITY_INT_PIN, true, false, (GPIOINT_IrqCallbackPtr_t) GPIO_int_callback);
	set_gpio_interrupt(gpioPortB,  FREE_FALL_INT_PIN, true, false, (GPIOINT_IrqCallbackPtr_t) GPIO_int_callback);

	while (1)
	{
		EMU_EnterEM2(false);

		// Wait a second
		delay(1000);

		// Clear interrupts by reading the INT_SOURCE register
		i2c_read_register(ADXL345_REG_INT_SOURCE);

		// Clear the LED for both (could clobber one that just came in)
		set_led(1,0);
		set_led(0,0);
	}
}
