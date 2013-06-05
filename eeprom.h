#ifndef EEPROM_H
#define EEPROM_H

void writeEEPROM(char data, char addressH, char addressL);
char readEEPROM(char addressH, char addressL);
void testEEPROM(void);
void addNewData(char data);
void writeEEPROMTestData(void);
void updateMapData(char virtualW, char virtualS,char virtualE, char virtualN, char victim, char move);
void EEPROMToSerial(void);

#define WREN  6 //Set Write Enable Latch
#define WRDI  4 //Reset Write Enable Latch
#define RDSR  5 //Read Status Register
#define WRSR  1 //Write Status Register
#define READ  3 //Read Data from Memory Array
#define WRITE 2 //Write Data to Memory Array
#endif