#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "lcd.h"

//write controls to LCD
void lcd_write_control(unsigned char databyte)
{
	EN = 0;
	RW = 0;
	RS = 0;
	PORTD = databyte;
	EN = 1;
	EN = 0;
	__delay_ms(2);
}

//write data to LCD
void lcd_write_data(unsigned char databyte)
{
	EN = 0;
	RW = 0;
	RS = 1;
	PORTD = databyte;
	EN = 1;
	EN = 0;
	__delay_ms(1);
}

//move the LCD cursor to a particular location
void lcd_set_cursor(unsigned char address)
{
	address |= 0b10000000;		//format address command using mask
	lcd_write_control(address);	//write address command
}

void lcd_write_string(const char * s)
{
	// write characters
	while(*s) lcd_write_data(*s++);
}

void lcd_write_1_digit_bcd(unsigned char data)
{
	lcd_write_data(data + 48);
}

//function accepts char between 0 and 99 and writes it to lcd display in 2 digits
void lcd_write_3_digit_bcd(int data)
{
	int OnesDigit, TensDigit, HundredsDigit;

	//load number to be converted into OnesDigit and clear TensDigit
	OnesDigit = data;
	TensDigit = 0;
	HundredsDigit = 0;

	//Perform a BCD Conversion	
	while (OnesDigit >= 10)
	{
		OnesDigit -= 10;
		TensDigit++;
	}

	while (TensDigit >= 10)
	{
		TensDigit -= 10;
		HundredsDigit++;
	}

	lcd_write_data((unsigned char)HundredsDigit + 48);
	lcd_write_data((unsigned char)TensDigit + 48);
	lcd_write_data((unsigned char)OnesDigit + 48);
}

//function initalises the LCD module - check that ADCON1 setting doesn't conflict
void lcd_init(void)
{


	//setup ADCON1 register to make PortE Digital
	ADCON1 = 0b00000010;	//left justified, PortE Digital, PortA Analogue


	PORTD = 0;				//set all pins on portd low
	PORTE = 0;				//set all pins on porte low

	TRISD = 0b00000000;		//set all pins to output
	TRISE = 0b00000000;		//set all pins to output

	//LCD Initialisation
	lcd_write_control(0b00000001); //clear display
	lcd_write_control(0b00111000); //set up display
	lcd_write_control(0b00001100); //turn display on
	lcd_write_control(0b00000110); //move to first digit
	lcd_write_control(0b00000010); //entry mode setup

}

#endif