#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

/********** #INCLUDES **********/

#include <htc.h>
#include "adc.h"
#include "drive.h"
#include "eeprom.h"
#include "ir.h"
#include "lcd.h"
#include "main.h"
#include "map.h"
#include "sensors.h"
#include "ser.h"
#include "songs.h"
#include "xtal.h"

/********** CONFIG **********/

__CONFIG(FOSC_HS &  WDTE_OFF & CP_OFF & BOREN_OFF & PWRTE_ON & WRT_OFF & LVP_OFF & CPD_OFF);

/* PUT #DEFINES AND PROTOTYPES IN HEADER FILE *************************************/

/********** GLOBAL VARIABLES **********/

volatile bit RTC_FLAG_1MS = 0;
volatile bit RTC_FLAG_10MS = 0;
volatile bit RTC_FLAG_50MS = 0;
volatile bit RTC_FLAG_500MS = 0;
volatile bit goingHome, home = FALSE;
volatile bit rightWall, frontWall, leftWall = 0;
volatile bit victimFound = FALSE;

volatile unsigned char victimZone = 0;
volatile unsigned char xCoord = 1;
volatile unsigned char yCoord = 3;
volatile unsigned char node = 0;

volatile unsigned int RTC_Counter = 0;


PushButton start;

/********** INTERRUPT SERVICE ROUTINE **********/
void interrupt isr1(void) 
{
	//Timer 1
	if(TMR0IF)
	{
		TMR0IF = 0;
		TMR0 = TMR0_VAL;
		
		RTC_Counter++;
		//set clock flags
		RTC_FLAG_1MS = 1;

		if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
		if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
		if(RTC_Counter % 500 == 0) 
		{

		}		

		if(START_PB)																			// If PB1 has been pressed
		{
			start.debounceCount++;															// Increment PB1 debounce count
			if(start.debounceCount >= DEBOUNCE_REQ_COUNT & start.released)						// If signal has been debounced sufficiently and switch has been released
			{
				start.pressed = TRUE;															/* PB1 has been pressed       */
				start.released = FALSE;														/* and therefore not released */
			}
		}
		else																				// If PB1 has not been pressed
		{
			start.debounceCount = 0;															// Set PB1 debounce count to 0
			start.released = TRUE;															// PB1 is therefore released
		}

		ser_int();
	}
}	

/******** INITIALISTIONS ********/

void init()
{
	start.pressed = FALSE;														/* Initialise all push buttons to not being pressed */
	start.released = TRUE;														/* but rather released                              */
	
	init_adc();
	lcd_init();
	
	TRISB = 0b00000001; 																	/* PORTB I/O designation */

	//timer0
	OPTION_REG = 0b00000100;
	//enable timer0 interrupt
	TMR0IE = 1;
	SSPSTAT = 0b01000000;																	/* Transmit on active -> idle state, sample input at middle of output */
	SSPCON = 0b00100010;																	/* Enable serial port, Fosc/64                                        */
	TRISC = 0b10010000;																		/* RX and SPI_SDI inputs, rest outputs                                */
	PORTC = 0b00000000;																		/* No module selected, no SM pulse, etc.                              */
	
	//Enable all interrupts
	PEIE = 1;	
	GIE  = 1;

	ser_init(); 																			//initialize UART
	initIRobot();
	initSongs();
}

void initIRobot()
{
	__delay_ms(100);
	ser_putch(128);
	ser_putch(132);
}

/********** FUNCTIONS **********/

void checkForFinalDestination()
{
	if(!goingHome && (xCoord == getFinalX()) && (yCoord == getFinalY()))
	{		
		play_iCreate_song(2);
		goingHome = TRUE;
		lcd_set_cursor(0x06);
		lcd_write_data('R');
	}
}

void lookForVictim()
{
	//request home base data
	if(victimFound)
	{
		if(goingHome)
		{
			play_iCreate_song(3);
			victimZone = 0;
			lcd_set_cursor(0x09);
			lcd_write_data('V');
		}
		else
		{
			victimZone = getVictimZone(xCoord, yCoord);
			lcd_set_cursor(0x08);
			lcd_write_1_digit_bcd(victimZone);
		}
	}
}

// Finds where there are walls around the Create's location
void findWalls()
{
	lcd_set_cursor(0x0B);

	leftWall = findWall();
	if(leftWall)
	{
		lcd_write_data('L');
	}
	else
		lcd_write_data(' ');
	rotateIR(24, CW); // Rotate 180deg CCW

	frontWall = findWall();
	if(frontWall)
	{
		lcd_write_data('F');
		frontWallCorrect();
	}
	else
		lcd_write_data(' ');
	rotateIR(24, CW); // Rotate 180deg CCW

	rightWall = findWall();
	if(rightWall)
	{
		lcd_write_data('R');
		rightWallCorrect();
	}
	else
		lcd_write_data(' ');
	rotateIR(48, CCW); // Rotate 90deg CW		
}

void goToNextCell()
{
	if(!leftWall)
		goLeft();
	else if(!frontWall)
		goForward();
	else if(!rightWall)
		goRight();
	else
		goBackward();
}

void updateLocation()
{
	lcd_set_cursor(0x40);
	switch(getOrientation())
	{
		case NORTH:
			++yCoord;
			lcd_write_data('N');		
			break;
		case SOUTH:
			--yCoord;
			lcd_write_data('S');
			break;
		case EAST:
			++xCoord;
			lcd_write_data('E');
			break;
		case WEST:
			--xCoord;
			lcd_write_data('W');
			break;
		default:
			break;
	}

	lcd_set_cursor(0x01);
	lcd_write_1_digit_bcd(xCoord);
	lcd_set_cursor(0x03);
	lcd_write_1_digit_bcd(yCoord);	
}

void updateNode()
{
	if((xCoord == 2) && (yCoord == 2))	
		node = 1;
	else if((xCoord == 4) && (yCoord == 2))
		node = 2;
	else if((xCoord == 2) && (yCoord == 0))
		node = 3;
	else
		node = 0;
}

void checkIfHome()
{
	if((xCoord == 1) && (yCoord == 3))
	{		
		STOP();
		play_iCreate_song(4);
		home = TRUE;
	}	
}

/************ MAIN ************/

void main(void)
{
	init();
	STOP();

	lcd_set_cursor(0x00);
	lcd_write_string("(-,-) E -- --- -"); //x/y of robot, explore/return, zone of victim/got victim, walls, way went
	lcd_set_cursor(0x40);
	lcd_write_string("- - - (0,0) GREG"); //orientation, cliff detected, v.wall detected, x/y of final destination

	//play_iCreate_song(1);

	while(!home)
	{
		if(start.pressed)
		{
			checkForFinalDestination();
			lookForVictim();
			findWalls();
			switch(node)
			{
				case 0:
					goToNextCell();
					break;
				case 1:
					goToNextCell();
					break;
				case 2:
					goToNextCell();
					break;
				case 3:
					goToNextCell();
					break;
				default:
					break;
			}
			//sendEEPROMData();
			updateLocation();
			updateNode();
			if(goingHome)
				checkIfHome();
		}
	}
	//Turn off
}

/******** SUB-FUNCTIONS ********/

// Returns true if the IR sensor detects something less than 100cm away
bit findWall()
{
	if(readIR() > 100)
		return FALSE;
	else
		return TRUE;
}

char getCurrentX()
{
	return xCoord;
}

char getCurrentY()
{
	return yCoord;
}

#endif