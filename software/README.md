


Change in Makefile:

software/shakti-sdk/software/examples/Makefile

ifeq ($(PROGRAM),serialCommunication)
filepath := uart_applns/serialCommunication
else
        @echo "Entry for $(PROGRAM) not present"
endif


entry 
@echo "Entry for $(PROGRAM) not present"
already exist, replace that with the 5 lines above



bsp/include/gpio_i2c.h:
#define I2C_SCL GPIO2
#define I2C_SLA GPIO3


