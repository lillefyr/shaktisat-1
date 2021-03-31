/**************************************************************************
 * Project           	         : shakti devt board
 * Name of the file	             : Wire.cpp
 * Brief Description of file     : cpp file for Wire
 * Name of Author    	         : Mainak Mondal & Sambhav Jain
 * Email ID                      : mainak19981998@gmail.com
 
 Copyright (C) 2019  IIT Madras. All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
*****************************************************************************/

//#include "Arduino.h"
#include "Wire.h"
extern "C"
{
#include "i2c.h"
#include "utils.h"
#include "log.h"
}

// Initialize Class Variables //////////////////////////////////////////////////

char rxBuffer[BUFFER_LENGTH];
int rxBufferIndex = 0;
int rxBufferLength = 0;
int txAddress = 0;
int txBuffer[BUFFER_LENGTH];
int txBufferIndex = 0;
int txBufferLength = 0;
#define TWI_BUFFER_LENGTH 32
static volatile int twi_txBufferLength;
static int twi_txBuffer[TWI_BUFFER_LENGTH];
int transmitting = 0;
int SLAVE_ADDRESS;
unsigned int *readTemp;
unsigned char length;
//#define CONTROL_REGISTER_VALUE 0x04
#define REG_OFFSET 0
//#define DELAY_VALUE 900
#define PRESCALER_COUNT 0x1F
#define SCLK_COUNT 0x91
#define I2C i2c_instance[1]
unsigned char i = 0;
unsigned long delay1 = 1000;
//unsigned int write_buf[7] = {0x00}, read_buf[7] = {0x00};

void TwoWire::begin()
{
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;
	i2c_init();

	if (config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT))
	{
		printf("\tSomething Wrong In Initialization\n");
		//return 0;
	}
	else
	{
		printf("\tInitialization Happened Fine\n");
	}
}

void TwoWire::begin(int address)
{
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;
	i2c_init();

	if (config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT))
	{
		printf("\tSomething Wrong In Initialization\n");
		//return 0;
	}
	else
	{
		printf("\tInitialization Happened Fine\n");
	}
	SLAVE_ADDRESS = (address << 1);
}

///////////////////////
int TwoWire::requestFrom(int address, int quantity, long iaddress, int isize, int sendStop)
{
	//	printf("Debugging  requestFrom \n");
	if (isize > 0)
	{
		// send internal address; this mode allows sending a repeated start to access
		// some devices' internal registers. This function is executed by the hardware
		// TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

		beginTransmission((address << 1));

		// the maximum size of internal address is 3 bytes 
		if (isize > 3)
		{
			isize = 3;
		}

		// write internal register address - most significant byte first
		while (isize-- > 0)
			write((int)(iaddress >> (isize * 8)));
		endTransmission(false);
	}

	// clamp to buffer length
	if (quantity > BUFFER_LENGTH)
	{
		quantity = BUFFER_LENGTH;
	}
	// perform blocking read into buffer
	i2c_send_slave_address(I2C, (address << 1), I2C_READ, delay1);
	int read = readbytes(I2C, rxBuffer, quantity, sendStop);
	//printf("Read %d number of bytess",read);
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = read;

	return read;
}

int TwoWire::requestFrom(int address, int quantity, int sendStop)
{
	return requestFrom((int)address, (int)quantity, (long)0, (int)0, (int)sendStop);
}

int TwoWire::requestFrom(int address, int quantity)
{
	return requestFrom((int)address, (int)quantity, (int) true);
}

int TwoWire::available(void)
{
	return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int TwoWire::read(void)
{
	int value = -1;

	// get each successive byte on each call
	if (rxBufferIndex < rxBufferLength)
	{
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}

	return value;
}

void TwoWire::beginTransmission(int address)
{
	// indicate that we are transmitting
	transmitting = 1;
	// set address of targeted slave
	txAddress = (address << 1);
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
}

//   unsigned int reg_offset, unsigned int *write_value, unsigned char length, unsigned long delay1
	/*  
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 *          5 .. timeout
 * */
int TwoWire::endTransmission(int sendStop)
{
	if (TWI_BUFFER_LENGTH < length)
	{
		return 1;
	}
	unsigned int reg_offset = REG_OFFSET;
	int *write_value = txBuffer;
	unsigned int txLength = txBufferLength;
	int i = 0, j = 0, k = 0, status = 0;
	unsigned int temp = 0;
	//	printf("\nEnd Transmission Function - %x %x\n",txLength,txAddress);
	int ret_i2c = i2c_send_slave_address(I2C, txAddress, I2C_WRITE, delay1);
	//i2c_write_data(I2C, txBuffer[0], delay1);
	//	printf("EndTransmissino probe 2\n");
	for (i = 0; i < txLength; i++)
	{
		i2c_write_data(I2C, txBuffer[i], delay1);
	}
	//printf("EndTransmissino probe 3\n");
	if (sendStop)
	{
		//Stops the I2C transaction to start reading the temperature value.
		I2C->control = I2C_STOP;
	}
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
	// indicate that we are done transmitting
	transmitting = 0;
	if (ret_i2c == -3)
		return 5; // success
	else if (ret_i2c == -1)
		return 2; // error: address send, nack received
	else if (ret_i2c == -2)
		return 3; // error: data send, nack received
	else if (ret_i2c == -4)
		return 4; // other twi error
	else
		return 0;
}
//////////////////////////////
//TOdo
void TwoWire::endTransmission(void)
{
	endTransmission(true);
}

int twi_transmit(const int *data, int length)
{
	int i;
	// ensure data will fit into buffer
	if (TWI_BUFFER_LENGTH < (twi_txBufferLength + length))
	{
		return 1;
	}
	// set length and copy data into tx buffer
	for (i = 0; i < length; ++i)
	{
		twi_txBuffer[twi_txBufferLength + i] = data[i];
	}
	twi_txBufferLength += length;
	return 0;
}
int TwoWire::write(int data)
{
	if (transmitting)
	{
		// in master transmitter mode
		// don't bother if buffer is full
		if (txBufferLength >= BUFFER_LENGTH)
		{
			//setWriteError();
			// Mainak Need to implement - check setWriteError function
			return 0;
		}
		// put byte in tx buffer
		//	printf("txBufferLength %d txBufferIndex %d\n",txBufferLength,txBufferIndex);
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		// update amount in buffer
		txBufferLength = txBufferIndex;
		//printf("txBufferLength2 %d txBufferIndex2 %d\n",txBufferLength,txBufferIndex);
	}
	else
	{
		// in slave send mode
		// reply to master
		twi_transmit(&data, 1);
	}
	return 1;
}

///////////////////
int TwoWire::write(const int *data, int quantity)
{
	if (transmitting)
	{
		// in master transmitter mode
		for (int i = 0; i < quantity; ++i)
		{
			write(data[i]);
		}
	}
	else
	{
		// in slave send mode
		// reply to master
		twi_transmit(data, quantity);
	}
	return quantity;
}

//////////////////////////

void TwoWire::setClock(long)
{
}

void TwoWire::onReceive(void (*)(int))
{
}

void TwoWire::onRequest(void (*)(void))
{
}

//////////////////////////
TwoWire Wire = TwoWire();
