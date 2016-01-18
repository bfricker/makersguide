#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_i2c.h"
#include "utilities.h"
#include "adxl.h"

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

// Write a config register on an I2C device
// Tailored for the ADX345 device only i.e. 1 byte of TX
void i2c_write_register(uint8_t reg_offset, uint8_t write_data)
{
	cmd_array[0] = reg_offset;
	data_array[0] = write_data;
	i2c_transfer(ADXL345_ADDRESS, cmd_array, data_array, 1, 1, I2C_FLAG_WRITE_WRITE);
}

void i2c_setup()
{
	CMU_ClockEnable(cmuClock_I2C0, true);

	I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

	init.enable                    = 1;
	init.master                    = 1;
	init.freq                      = I2C_FREQ_STANDARD_MAX;
	init.clhr                      = i2cClockHLRStandard;
	I2C_Init(I2C0, &init);

	// The rest of this is cut-and-pasted from chapter 10's InitDevice.c file
	/* Module I2C0 is configured to location 1 */
	I2C0->ROUTE = (I2C0->ROUTE & ~_I2C_ROUTE_LOCATION_MASK) | I2C_ROUTE_LOCATION_LOC1;

	/* Enable signals SCL, SDA */
	I2C0->ROUTE |= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN;

	/* Module PCNT0 is configured to location 1 */
	PCNT0->ROUTE = (PCNT0->ROUTE & ~_PCNT_ROUTE_LOCATION_MASK) | PCNT_ROUTE_LOCATION_LOC1;

	/* Module USART1 is configured to location 1 */
	USART1->ROUTE = (USART1->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC1;

	/* Enable signals RX, TX */
	USART1->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;
	// [Route Configuration]$


	/* Pin PD6 is configured to Open-drain with pull-up and filter */
	GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_WIREDANDPULLUPFILTER;

	/* Pin PD7 is configured to Open-drain with pull-up and filter */
	GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK) | GPIO_P_MODEL_MODE7_WIREDANDPULLUPFILTER;
	// [Port D Configuration]$
}

// Turns on interrupts, sets up the thresholds, and enables interrupts on the ADXL345
void i2c_init_registers()
{
	// Set range to 16g and FULL resolution
	i2c_write_register(ADXL345_REG_DATA_FORMAT, ADXL345_RANGE_16_G );

	// ADXL_BW_RATE = 50 hz, limited by I2C speed
	i2c_write_register(ADXL345_REG_BW_RATE, ADXL345_DATARATE_50_HZ);

	// Set up threshold for activity, found through trial and error
	i2c_write_register(ADXL345_REG_THRESH_ACT, 0x90);

	// Turn on the axes that will participate
	i2c_write_register(ADXL345_REG_ACT_INACT_CTL, ADXL345_ACT_ac_dc | ADXL345_ACT_X | ADXL345_ACT_Y | ADXL345_ACT_Z);

	// Set up interrupt outputs, sent to INT1 pin by default
	i2c_write_register(ADXL345_REG_INT_ENABLE, ADXL345_INT_Activity );

	uint32_t value = i2c_read_register(ADXL345_REG_INT_ENABLE);

	// Clear interrupts by reading the INT_SOURCE register
	i2c_read_register(ADXL345_REG_INT_SOURCE);

	// Start measurement
	i2c_write_register(ADXL345_REG_POWER_CTL, 0x08);
}
