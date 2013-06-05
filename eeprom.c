#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include <htc.h>
#include "xtal.h"
#include "lcd.h"
#include "eeprom.h"
#include "ser.h"

char addressCount = 0;
char addressCurrent = 0;

void writeSPIByte(unsigned char data)
{
    SSPIF = 0;
    SSPBUF = data;
    while(!SSPIF);
}

void initEEPROMMode(void)
{
    PORTC &= 0b11111100; // Set CPLD to EEPROM Mode  
    PORTC |= 0b00000010; // Set CPLD to EEPROM Mode
}

void writeEEPROM(unsigned char data, unsigned char addressH, unsigned char addressL)
{  
  
    __delay_ms(100);
    initEEPROMMode();
    //EEPROM write enable instruction
    writeSPIByte(WREN);
    initEEPROMMode();
  
    //Write to EEPROM instruction
    writeSPIByte(WRITE);

    //Address High Byte
    writeSPIByte(addressH);

    //Address Low Byte
    writeSPIByte(addressL);

    //Write Char
    writeSPIByte(data);
    initEEPROMMode();
}

unsigned char readEEPROM(unsigned char addressH, unsigned char addressL)
{
    __delay_ms(100);
    initEEPROMMode();
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

//Test EEPROM
void testEEPROM(void)
{
    const char * text1 = "EEPROM Success";
    const char * text2 = "EEPROM Failed";
  
    writeEEPROM('P',0x00,0x55);
    writeEEPROM('E',0x00,0x56);

    unsigned char test1 = readEEPROM(0x00,0x55);
    unsigned char test2 = readEEPROM(0x00,0x56);

    if(test1 == 'P' && test2 == 'E')
    {
        lcd_write_string(text1);
    }else
    {
        lcd_write_string(text2);
    }
    __delay_ms(5000);
}

//Express way of adding new data to EEPROM
void addNewData(char data)
{
	writeEEPROM(data,0,addressCount + 1);
	addressCount ++;
	writeEEPROM(addressCount,0,0);
}

//Write some EEPROM test data
//EMMVNESW
void writeEEPROMTestData(void)
{	
	addNewData(17);
	addNewData(32);
	addNewData(64);
	addNewData(64);
	addNewData(96);
	addNewData(32);
	addNewData(64);
	addNewData(64);
	addNewData(96);
	addNewData(0);
	addNewData(64);
	addNewData(32);
	addNewData(32);
	addNewData(32);
	addNewData(0);
	addNewData(0);
	addNewData(96);
	addNewData(64);
	addNewData(0);
	addNewData(32);
	addNewData(0);
	addNewData(96);
	addNewData(0);
	addNewData(32);
	addNewData(96);
	addNewData(96);	
}

//Create a new entry into EEPROM with appropriate flags set
//EMMVNESW
void updateMapData(char virtualW, char virtualS,char virtualE, char virtualN, char victim, char move)
{
	char completeData = 0;
	completeData |= virtualW;
	completeData |= virtualS << 1;
	completeData |= virtualE << 2;
	completeData |= virtualN << 3;
	completeData |= victim << 4;
	completeData |= move << 5;
	completeData &= 0b01111111;
	addNewData(completeData);
}

//Send the EEPROM data through serial connection
void EEPROMToSerial(void)
{
	char transferDone = 0;
	addressCurrent = 0;
	addressCount = readEEPROM(0,0);
	__delay_ms(100);
	//write the start char
	ser_putch(254);
	//transfer EEPROM data
	while(!transferDone && addressCount > 0)
	{
		ser_putch(readEEPROM(0,1 + addressCurrent));
		__delay_ms(100);
		addressCurrent ++;
		if(addressCurrent >= (addressCount))
		{
			transferDone = 1;
		}
	}
	//write the 2 end characters
	ser_putch(255);
	__delay_ms(100);
	ser_putch(255);
}

#endif