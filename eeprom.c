//#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include <htc.h>
#include "xtal.h"
#include "lcd.h"
#include "eeprom.h"

//Address format XX000000 00000000

void writeSPIByte(unsigned char data)
{
	SSPIF = 0;
	SSPBUF = data;
	while(!SSPIF);
}

void writeEEPROM(unsigned char data, unsigned char addressH, unsigned char addressL)
{	
	
	PORTC &= 0b11111100; // Set CPLD to EEPROM Mode	
	PORTC |= 0b00000010; // Set CPLD to EEPROM Mode
	//EEPROM write enable instruction
	writeSPIByte(WREN);

	PORTC &= 0b11111100; // Set CPLD to EEPROM Mode	
	PORTC |= 0b00000010; // Set CPLD to EEPROM Mode
	
	//Write to EEPROM instruction
	writeSPIByte(WRITE);

	//Address High Byte
	writeSPIByte(addressH);

	//Address Low Byte
	writeSPIByte(addressL);

	//Write Char
	writeSPIByte(data);
	PORTC &= 0b11111100; // Set CPLD to EEPROM Mode	
	PORTC |= 0b00000010; // Set CPLD to EEPROM Mode
}

unsigned char readEEPROM(unsigned char addressH, unsigned char addressL)
{
	unsigned char returnData = 0;
	
	writeSPIByte(READ);

	writeSPIByte(addressH);

	writeSPIByte(addressL);
	
	//get the data
	SSPIF = 0;
	SSPBUF = 0xFF;
	while (!SSPIF);	
	
	returnData = SSPBUF;

	return returnData;
}

void testEEPROM(void)
{
	const char * text = "EEPROM Worked";
	const char * text2 = "EEPROM Failed";
	
	unsigned char test = 'a';
	
	PORTC &= 0b11111100; // Set CPLD to EEPROM Mode	
	PORTC |= 0b00000010; // Set CPLD to EEPROM Mode
	__delay_ms(100);
	writeEEPROM('P',0x00,0x55);
	__delay_ms(100);

	PORTC &= 0b11111100; // Set CPLD to EEPROM Mode	
	PORTC |= 0b00000010; // Set CPLD to EEPROM Mode

	test = readEEPROM(0x00,0x55);
	lcd_write_data(test);
}

#endif