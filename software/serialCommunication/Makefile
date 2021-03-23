#Created by Sathya Narayanan N
# Email id: sathya281@gmail.com

#   Copyright (C) 2019  IIT Madras. All rights reserved.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

SHELL := /bin/bash # Use bash syntax
DC ?=
PROGRAM ?=
#default target board
TARGET ?= artix7_35t
DEBUG ?=
MARCH ?= rv32imac
MABI  ?= ilp32
XLEN  ?= 32
FLAGS ?=

all: create_dir
	make serialCommunication.riscv

serialCommunication.riscv: create_dir
	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(flags) -I$(bspboard) -I$(bspinc) -c ./serialCommunication.c -o ./output/serialCommunication.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(flags) -I$(bspboard) -I$(bspinc) -c ./MPU6050.c -o ./output/MPU6050.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(flags) -I$(bspboard) -I$(bspinc) -c ./HMC5883.c -o ./output/HMC5883.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(flags) -I$(bspboard) -I$(bspinc) -c ./BME280.c -o ./output/BME280.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(flags) -I$(bspboard) -I$(bspinc) -c ./DS3231.c -o ./output/DS3231.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) -I$(bspboard) -I$(bspinc) -c $(bspdri)/i2c/i2c_driver.c -o ../../gen_lib/i2c_driver.o -lm -lgcc

	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) -T  $(bspboard)/link.ld $(GENLIB)/gen_lib/libshakti$(XLEN).a ./output/serialCommunication.o ./output/MPU6050.o ./output/HMC5883.o ./output/BME280.o ./output/DS3231.o ../../gen_lib/i2c_driver.o -o ./output/$(PROGRAM).shakti -static -nostartfiles -lm -lgcc

	@riscv$(XLEN)-unknown-elf-objdump -D ./output/$(PROGRAM).shakti > ./output/$(PROGRAM).dump

create_dir:
	@mkdir -p ./output
