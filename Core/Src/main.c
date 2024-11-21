#include "main.h"

// LCD
#define LCD_ADDR  			0x27			// LCD Address
#define LCD_EN 	  			0x04			// Enable
#define LCD_RW    			0x02			// Read/Write
#define LCD_RS    			0x01			// Register Select
#define LCD_BL	  			0x08			// Back-light

// LCD Commands
#define LCD_CLEAR 			0x01			// Clear Screen
#define LCD_HOME  			0x02			// Cursor Home
#define LCD_ENTRY_MODE  	0x06			// Increments cursor, no shift
#define LCD_DISPLAY_ON  	0x0C			// Display on, cursor off, blink off
#define LCD_FUNCTION_SET 	0x28			// 4-bit mode, 2 lines, 5x8 font

// Prototypes
void I2C_Write(uint8_t, uint8_t);
void SYS_Init();
void LCD_SendCommand(uint8_t);
void LCD_Init();
void LCD_SendData(uint8_t);
void LCD_SendString(char*);

int main(void) {
    SYS_Init();
    LCD_Init();

    for (volatile int i = 0; i < 100000; i++);
    LCD_SendString("Hello World!");

    while(1) {

    }
}

void GPIO_Init() {
	RCC -> AHB1ENR |= (1 << 1);											// Enable GPIOB Clock

	GPIOB -> MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2))); 				// Clears mode bits for PB8 and PB9
	GPIOB -> MODER |= (2 << (8 * 2)) | (2 << (9 *2));					// Sets mode to Alternate Function

	GPIOB -> OTYPER |= (1 << 8) | (1 << 9);								// Turns on Open-Drain for PB8 and PB9

	GPIOB -> PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));				// Clears Pull-Up/Pull-Down Register
	GPIOB -> PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));					// Sets pins to Pull-Up

	GPIOB -> AFR[1] &= ~(0xF << 0) | (0xF << 4);						// Clears Alternate Function for PB8 and PB9
	GPIOB -> AFR[1] |= (4 << 0) | (4 << 4);								// Sets the Alternate Function to AF4 for PB8 and PB9
}

void I2C_Init() {
	RCC -> APB1ENR |= (1 << 21); 										// Enable I2C Clock

	I2C1 -> CR1 |= (1 << 15);											// Toggles SWRST; Resets the I2C connection
	I2C1 -> CR1 &= ~(1 << 15);											// Disables SWRST; I2C is ready

	I2C1 -> CR2 |= (42 << 0);											// Sets clock frequency to 42MHz

	I2C1 -> CCR = 210;													// Configured for 100kHz; Standard frequency

	I2C1 -> TRISE = 43;													// Frequency + 1; Used to set maximum rise time

	I2C1 -> CR1 |= (1 << 0);											// Enables I2C
}

void SYS_Init() {
	GPIO_Init();
	I2C_Init();
}


uint8_t I2C_CheckBusy() {
	if (I2C1 -> SR1 & (1 << 1)) {
		return 1;  														// Bus is busy
	}
	return 0;															// Bus is free
}


void I2C_Start() {
	I2C1 -> CR1 |= (1 << 8); 											// Generates Start condition

	while (!(I2C1 -> SR1 & (1 << 0)));									// Waits for condition to generate
}


void I2C_SendAddress(uint8_t address, uint8_t read) {
	I2C1 -> DR = (address << 1) | read;									// Sets the data register to the read/write bit along with the address of the device

	while (!(I2C1 -> SR1 & (1 << 1)));									// Waits until address is sent

	uint8_t dummyRead = I2C1 -> SR1;
	dummyRead = I2C1 -> SR2;
	(void)dummyRead;													// Acknowledges the address is sent and void to ignore warning
}

void I2C_SendData(uint8_t data) {
	while (!(I2C1 -> SR1 & (1 << 7)));									// Waits for data register to be empty

	I2C1 -> DR = data;													// Inputs data to data register

	while (!(I2C1 -> SR1 & (1 << 2)));									// Waits until transfer is finished
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

void LCD_SendCommand(uint8_t cmd) {
	uint8_t upper = (cmd & 0xF0) | LCD_BL;								// Upper nibble
	uint8_t lower = ((cmd << 4) & 0xF0) | LCD_BL;						// Lower nibble

	I2C_Write(LCD_ADDR,	upper | LCD_EN);								// Sends upper; pulses
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_EN);								// Sends lower; pulses
	I2C_Write(LCD_ADDR, lower);
}

void LCD_Init() {
	for (volatile int i = 0; i < 50000; i++);

	LCD_SendCommand(0x33);												// Initializes LCD
	LCD_SendCommand(0x32);												// Sets function to 4-bit mode
	LCD_SendCommand(LCD_FUNCTION_SET);									// Configures 4-bit mode
	LCD_SendCommand(LCD_DISPLAY_ON);									// Turns on display
	LCD_SendCommand(LCD_ENTRY_MODE);									// Sets entry mode
	LCD_SendCommand(LCD_CLEAR);											// Clears screen
}

void LCD_SendData(uint8_t data) {
	uint8_t upper = (data & 0xF0) | LCD_BL | LCD_RS;					// Upper nibble; register bit is set to set data instead of command
	uint8_t lower = ((data << 4) & 0xF0) | LCD_BL | LCD_RS;				// Lower nibble; register bit is set to set data instead of command

	I2C_Write(LCD_ADDR, upper | LCD_EN);								// Sends upper; pulses
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_EN);								// Sends lower; pulses
	I2C_Write(LCD_ADDR, lower);
}


// Users a character pointer to print a string
void LCD_SendString(char *str) {
    while(*str != '\0') {
        LCD_SendData(*str);
        str++;
    }
}























