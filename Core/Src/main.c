#include "main.h"
#include <stdio.h>

void I2C_Write(uint8_t address, uint8_t data);
void SYS_Init();

int main(void) {
    SYS_Init();

    while(1) {
    }
}

void GPIO_Init() {
	RCC -> AHB1ENR |= (1 << 1);											// Enable GPIOB Clock

	GPIOB -> MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2))); 								// Clears mode bits for PB8 and PB9
	GPIOB -> MODER |= (2 << (8 * 2)) | (2 << (9 *2));								// Sets mode to Alternate Function

	GPIOB -> OTYPER |= (1 << 8) | (1 << 9);										// Turns on Open-Drain for PB8 and PB9

	GPIOB -> PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));								// Clears Pull-Up/Pull-Down Register
	GPIOB -> PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));								// Sets pins to Pull-Up

	GPIOB -> AFR[1] &= ~(0xF << 0) | (0xF << 4);									// Clears Alternate Function for PB8 and PB9
	GPIOB -> AFR[1] |= (4 << 0) | (4 << 4);										// Sets the Alternate Function to AF4 for PB8 and PB9
}

void I2C_Init() {
	RCC -> APB1ENR |= (1 << 21); 											// Enable I2C Clock

	I2C1 -> CR1 |= (1 << 15);											// Toggles SWRST; Resets the I2C connection
	I2C1 -> CR1 &= ~(1 << 15);											// Disables SWRST; I2C is ready

	I2C1 -> CR2 |= (42 << 0);											// Sets clock frequency to 42MHz

	I2C1 -> CCR = 210;												// Configured for 100kHz; Standard frequency

	I2C1 -> TRISE = 43;												// Frequency + 1; Used to set maximum rise time

	I2C1 -> CR1 |= (1 << 0);											// Enables I2C
}

void SYS_Init() {
	GPIO_Init();
	I2C_Init();
}


uint8_t I2C_CheckBusy() {
	if (I2C1 -> SR1 & (1 << 1)) {
		return 1;  												// Bus is busy
	}
	return 0;													// Bus is free
}


void I2C_Start() {
	I2C1 -> CR1 |= (1 << 8); 											// Generates Start condition

	while (!(I2C1 -> SR1 & (1 << 0)));										// Waits for condition to generate
}


void I2C_SendAddress(uint8_t address, uint8_t read) {
	I2C1 -> DR = (address << 1) | read;										// Sets the data register to the read/write bit along with the address of the device

	while (!(I2C1 -> SR1 & (1 << 1)));										// Waits until address is sent

	uint8_t dummyRead = I2C1 -> SR1;
	dummyRead = I2C1 -> SR2;
	(void)dummyRead;												// Acknowledges the address is sent and void to ignore warning
}

void I2C_SendData(uint8_t data) {
	while (!(I2C1 -> SR1 & (1 << 7)));										// Waits for data register to be empty

	I2C1 -> DR = data;												// Inputs data to data register

	while (!(I2C1 -> SR1 & (1 << 2)));										// Waits until transfer is finished
}

void I2C_Stop() {
	I2C1 -> CR1 |= (1 << 9);											// Generates stop condition
}

void I2C_Write(uint8_t address, uint8_t data) {
	I2C_CheckBusy();
	I2C_Start();
	I2C_SendAddress(address, 0);
	I2C_SendData(data);
	I2C_Stop();
}



