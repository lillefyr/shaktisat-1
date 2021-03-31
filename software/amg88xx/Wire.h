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
#ifndef Wire_h
#define Wire_h

#define BUFFER_LENGTH 32

class TwoWire
{
public:
	void begin();
	void begin(int address);

	int requestFrom(int address, int quantity, long iaddress, int isize, int sendStop); //
	int requestFrom(int address, int quantity, int sendStop);
	int requestFrom(int address, int quantity);

	void beginTransmission(int address); //

	void endTransmission(void);			   //
	int endTransmission(int sendStop); //

	int write(int data);							//
	int write(const int *data, int quantity); //

	int available(void); //

	int read(void);

	void setClock(long);

	void onReceive(void (*)(int));

	void onRequest(void (*)(void));
};

extern TwoWire Wire;
#endif