#ifndef	LCD_H
#define LCD_H

#include <htc.h>
#include "xtal.h"

//defines
#define RS RE0			//LCD Register Select
#define RW RE1			//LCD Read Write
#define EN RE2			//LCD Enable


//write controls to LCD
void lcd_write_control(unsigned char databyte);

//write data to LCD
void lcd_write_data(unsigned char databyte);

//move the LCD cursor to a particular location
void lcd_set_cursor(unsigned char address);

void lcd_write_string(const char * s);

void lcd_write_1_digit_bcd(unsigned char data);

//function accepts char between 0 and 99 and writes it to lcd display in 2 digits
void lcd_write_3_digit_bcd(int data);

//function initalises the LCD module - check that ADCON1 setting doesn't conflict
void lcd_init(void);


#endif //LCD_H