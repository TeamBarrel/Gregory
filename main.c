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

/********** GLOBAL VARIABLES **********/

volatile bit RTC_FLAG_1MS = 0;
volatile bit RTC_FLAG_10MS = 0;
volatile bit RTC_FLAG_50MS = 0;
volatile bit RTC_FLAG_500MS = 0;
volatile bit goingHome, home = FALSE;
volatile bit ready = FALSE;
volatile bit rightWall, frontWall, leftWall = 0;

volatile signed char xCoord = 1;
volatile signed char yCoord = 3;
volatile signed char xVictim = -10;
volatile signed char yVictim = -10;
volatile signed char xVirtual = -10;
volatile signed char yVirtual = -10;
volatile signed char dVirtual = WEST;

volatile unsigned char node = 0;
volatile unsigned char victimZone = 0;

volatile unsigned int RTC_Counter = 0;

PushButton start;
PushButton eepromSerial;

/********** INTERRUPT SERVICE ROUTINE **********/
void interrupt isr1(void) 
{
	//Timer 1
	if(TMR0IF)
	{
		TMR0IF = 0;
		TMR0 = TMR0_VAL;
	

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

		if(EEPROM_PB)																			// If PB1 has been pressed
		{
			eepromSerial.debounceCount++;															// Increment PB1 debounce count
			if(eepromSerial.debounceCount >= DEBOUNCE_REQ_COUNT & eepromSerial.released)						// If signal has been debounced sufficiently and switch has been released
			{
				eepromSerial.pressed = TRUE;															/* PB1 has been pressed       */
				eepromSerial.released = FALSE;														/* and therefore not released */
			}
		}
		else																				// If PB1 has not been pressed
		{
			eepromSerial.debounceCount = 0;															// Set PB1 debounce count to 0
			eepromSerial.released = TRUE;															// PB1 is therefore released
		}
		ser_int();
	}
}

/******** INITIALISTIONS ********/

void init()
{
	start.pressed = FALSE;														/* Initialise all push buttons to not being pressed */
	start.released = TRUE;														/* but rather released                              */
	eepromSerial.pressed = FALSE;
	eepromSerial.released = TRUE;
	
	init_adc();
	lcd_init();
	
	TRISB = 0b00000011; 																	/* PORTB I/O designation */

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

// Checks to see if the robot's current location is the pre-determined final destination
// If it is, set the goingHome bit and display 'return mode' on LCD
void checkForFinalDestination()
{
	if((xCoord == getFinalX()) && (yCoord == getFinalY()))
	{		
		play_iCreate_song(2);
		goingHome = TRUE;
		lcd_set_cursor(0x06);
		lcd_write_data('R');
	}
}

// Looks for any of the red buoy, green buoy, or force field emitted by the home base
// If the Create is in explore mode, remember and display the zone of the victim
// If the Create is in return mode, play a song and display victim found ('V'), and clear victim zone
void lookForVictim()
{
	ser_putch(142);
	ser_putch(17);
	char victim = ser_getch();
		
	if(victim > 241 && victim != 255)
	{
		if(goingHome)
		{
			__delay_ms(1000);
			play_iCreate_song(3);
			__delay_ms(500);
			victimZone = 0;
			lcd_set_cursor(0x09);
			lcd_write_data('V');
			xVictim = xCoord;
			yVictim = yCoord;
		}
		else
		{
			xVictim = xCoord;
			yVictim = yCoord;
			victimZone = getVictimZone(xCoord, yCoord);
			lcd_set_cursor(0x08);
			lcd_write_1_digit_bcd(victimZone);
		}
	}
}

// Finds where there are walls around the Create's location
void findWalls()
{
	rotateIR(24, CCW);	
	lcd_set_cursor(0x0B);

	leftWall = findWall();
	if(leftWall)
		lcd_write_data('L');
	else
		lcd_write_data(' ');

	rotateIR(24, CW);
	frontWall = findWall();

	if(frontWall)
		lcd_write_data('F');
	else
		lcd_write_data(' ');
	
	rotateIR(24, CW);
	rightWall = findWall();
	
	if(rightWall)
	{
		play_iCreate_song(5);
		lcd_write_data('R');
	}else
		lcd_write_data(' ');

	rotateIR(24, CCW);	
}

// Movement logic of the Create
// Preference is: Left, forward, right, backwards
void goToNextCell()
{
	if(!leftWall && (getSomethingInTheWay() != LEFT))
		goLeft();
	else if(!frontWall && (getSomethingInTheWay() != FORWARD))
		goForward();
	else if(!rightWall && (getSomethingInTheWay() != RIGHT))
		goRight();
	else
		goBackward();
}

// Updates the x-y co-ordinates of the Create
// The orientation is indicative of the way it last moved
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

// Updates the node the Create is on
void updateNode()
{
	if((xCoord == 2) && (yCoord == 2))	
		node = 1;
	else if((xCoord == 4) && (yCoord == 2))
		node = 2;
	else if((xCoord == 2) && (yCoord == 0))
		node = 3;		
	else if((xCoord == 4) && (yCoord == 3))
		node = 4;
	else if((xCoord == 2) && (yCoord == 1))
		node = 5;
	else if((xCoord == 3) && (yCoord == 0))
		node = 6;
	else
		node = 0;
}

// Plays a song if the Create is at its pre-determined starting location
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
	lcd_write_string("(-,-) - -- --- -"); //x/y of robot, explore/return, zone of victim/got victim, walls, way went
	lcd_set_cursor(0x40);
	lcd_write_string("- - - (3,1) GREG"); //orientation, cliff detected, v.wall detected, x/y of final destination
	char victimIndicator = 0;
	while(!home)
	{

		if(start.pressed && ready == FALSE)
		{
			findWalls();
			if(leftWall && rightWall && frontWall) 		//if facing east
				turnAround(); 							//turn to face west	
			else if (rightWall && frontWall)			//if facing north
				turnLeft90(); 							//turn to face west
			else if(leftWall && frontWall)				//if facing south
				turnRight90(); 							//turn to face west
			ready = TRUE; 								//commence exploring the map
			lcd_set_cursor(0x06);
			lcd_write_data('E');						//Explore mode
			play_iCreate_song(1);										
		}	
		

		//EEPROM Serial Transfer
		if(eepromSerial.pressed && ready == FALSE)
		{
			eepromSerial.pressed = FALSE;
			lcd_set_cursor(0x00);
			lcd_write_string("EEPROM Serial         ");
			lcd_set_cursor(0x40);
			lcd_write_string("Please Wait...        ");
			writeEEPROMTestData();
			EEPROMToSerial();
			lcd_set_cursor(0x40);
			lcd_write_string("Complete              ");
		}
		
		if(start.pressed)
		{
			ready = TRUE;
			checkForFinalDestination();

			lookForVictim();

			findWalls();
			play_iCreate_song(5);
			if(leftWall)
			{
				rotateIR(24,CCW);
				leftAngleCorrect();
				rotateIR(24,CW);
			}
			play_iCreate_song(5);
			if(frontWall)
				frontWallCorrect();
			play_iCreate_song(5);
			switch(node) // Gives the Create specific movement instructions for certain squares to go shortest path home and collecting the victim
			{
				case 0:
					goToNextCell();
					break;
				case 1:
					if (goingHome)
					{
						if (victimZone == 1)
							goRight();
						else if (getOrientation() == EAST)
							goForward();
						else if (getOrientation() == SOUTH)
							goRight();
						else
							goToNextCell();
					}
					else
						goToNextCell();
					break;
				case 2:
					if (goingHome)
					{
						if (victimZone == 2)
							goForward();
						else if (getOrientation() == SOUTH)
							goRight();
						else if (getOrientation() == NORTH)
							goLeft();
						else
							goToNextCell();
					}
					else
						goToNextCell();
					break;
				case 3:
					if (goingHome)
					{
						if (victimZone == 3)
							goRight();
						else if (getOrientation() == EAST)
							goForward();
						else if (getOrientation() == SOUTH)
							goLeft();
						else
							goToNextCell();
					}
					else
						goToNextCell();
					break;
				case 4:
					if (getOrientation() == EAST)
						goRight(); 
					else
						goToNextCell();
					break;
				case 5:
					if (getOrientation() == NORTH)
						goRight(); 
					else
						goToNextCell();
					break;
				case 6:
					if (getOrientation() == WEST)
					{
						play_iCreate_song(6);
						goForward();
					}
					else
						goToNextCell();
					break;
				default:
					break;
			}
			play_iCreate_song(5);
			if(getSuccessfulDrive())
			{
				//Send EEPROM data for current cell
				if(xVictim == xCoord && yVictim == yCoord)
				{
					victimIndicator = 1;
				}
				/*if(xVirtual == xCoord && yVirtual == yCoord)
				{
					switch(dVirtual)
					{
						case WEST:
						{
							updateMapData(1,0,0,0,victimIndicator,getOrientation());
							break;
						}
						case SOUTH:
						{
							updateMapData(0,1,0,0,victimIndicator,getOrientation());
							break;
						}
						case EAST:
						{
							updateMapData(0,0,1,0,victimIndicator,getOrientation());
							break;
						}
						case NORTH:
						{
							updateMapData(0,0,0,1,victimIndicator,getOrientation());
							break;
						}
						default:
							updateMapData(0,0,0,0,victimIndicator,getOrientation());
					}
				}*/
				
				updateMapData(0,0,0,0,victimIndicator,getOrientation());
				
				victimIndicator = 0;
				
				updateLocation();
				updateNode();		
				if(goingHome)
					checkIfHome();
			}
		}
	}
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

// Returns current x co-ordinate of the Create
char getCurrentX()
{
	return xCoord;
}

// Returns current y co-ordinate of the Create
char getCurrentY()
{
	return yCoord;
}

// Remembers the location and orientation of the virtual wall for EEPROM processing
void setVirtualLocation(char xV, char yV, char dV)
{
	xVirtual = xV;
	yVirtual = yV;
	dVirtual = dV;
}

#endif