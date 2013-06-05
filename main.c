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
volatile bit rightWall = 0;
volatile bit leftWall = 0;
volatile bit frontWall = 0;

volatile char walls[4] = {1,1,1,0};

volatile direction directionMoved;

volatile orientation currentOrientation = WEST;

volatile unsigned char xCoord = 1;
volatile unsigned char yCoord = 4;

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
			RTC_FLAG_500MS = 1;
			RTC_Counter = 0;	//reset RTC Counter
			//checkSensors();
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
	//initCellData();
	
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


// Returns true if the IR sensor detects something less than 100cm away
bit findWall()
{
	if(readIR() > 100)
		return FALSE;
	else
		return TRUE;
}

// Finds where there are walls around the Create's location
void findWalls()
{


	rotateIR(24, CW); // Rotate 90deg CW
	rightWall = findWall();
	rotateIR(24, CCW); // Rotate 180deg CCW
	frontWall = findWall();
	rotateIR(24, CCW); // Rotate 180deg CCW
	leftWall = findWall();
	rotateIR(24, CW); // Rotate 90deg CW
	
	int wallAtOrientation = 0;
	lcd_set_cursor(0x07);
	if(rightWall)
	{
		lcd_write_data('R');
		wallAtOrientation = RIGHT + currentOrientation;
		if(wallAtOrientation >= 4)
			wallAtOrientation -= 4;
		walls[wallAtOrientation] = 1;
	}
	else
	{
		lcd_write_data(' ');
	}
	if(frontWall)
	{
		lcd_write_data('F');
		walls[FORWARD+currentOrientation] = 1; // FORWARD = 0 therefore will never overflow
	}
	else
		lcd_write_data(' ');
	if(leftWall)
	{
		lcd_write_data('L');
		wallAtOrientation = LEFT + currentOrientation;
		if(wallAtOrientation >= 4)
			wallAtOrientation -= 4;
		walls[wallAtOrientation] = 1;
	}
	else
		lcd_write_data(' ');		
}


// Go one cell backwards
void goBackward()
{
	turnAround();
	driveForDistance(1000);
	directionMoved = BACKWARD;
	lcd_set_cursor(0x4F);
	lcd_write_data('B');
}

// Go one cell forwards
void goForward()
{
	driveForDistance(1000);
	directionMoved = FORWARD;
	lcd_set_cursor(0x4F);
	lcd_write_data('F');
}

// Go one cell left
void goLeft()
{
	turnLeft90();
	driveForDistance(1000);
	directionMoved = LEFT;
	lcd_set_cursor(0x4F);
	lcd_write_data('L');
}

// Determine which cell to go to next and go there
void goToNextCell()
{
	if(!rightWall)
		goRight();
	else if(!frontWall)
		goForward();
	else if(!leftWall)
		goLeft();
	else
		goBackward();
}

// Go one cell right
void goRight()
{
	turnRight90();
	driveForDistance(1000);
	directionMoved = RIGHT;
	lcd_set_cursor(0x4F);
	lcd_write_data('R');
}

void updateLocation()
{
	currentOrientation += directionMoved;

	if(currentOrientation >= 4)
		currentOrientation -= 4;

	lcd_set_cursor(0x46);
	switch(currentOrientation)
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

	lcd_set_cursor(0x0C);
	lcd_write_1_digit_bcd(xCoord);
	lcd_set_cursor(0x0E);
	lcd_write_1_digit_bcd(yCoord);
	
}

void main(void)
{
	init();
	STOP();
	//testEEPROM();
	//__delay_ms(5000);
	
	lcd_set_cursor(0x00);
	lcd_write_string("Walls@ --- (1,0)");
	lcd_set_cursor(0x40);
	lcd_write_string("cuOr: - dirMo: -");
	play_iCreate_song(4);
	while(1)
	{
		if(start.pressed)
		{
			//if(!isMoving())
			//{
			findWalls();
			//writeCellData();
			goToNextCell();
			updateLocation();
			//__delay_ms(5000);
			//}
		}
	}
}

#endif