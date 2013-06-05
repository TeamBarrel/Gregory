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
// Stepper motor
#define CW 0b00001111																		/* SSPBUF register to rotate CW */
#define CCW 0b00001101																		/*                      and CCW */
#define HALF_STEP_RESOLUTION 3.75
#define SCAN_IR_DEG(angle, motorDirection, updateClosestObject) scanIR((int)((angle)/HALF_STEP_RESOLUTION), (motorDirection), (updateClosestObject))
typedef struct {
	volatile char position;																			// Denotes position (in steps) of object
	volatile char distance;																			// Denotes distance (in cm) of object
} Object;
volatile char stepPosition = 0;											// Position of stepper motor (in CCW half-steps) since being turned on
volatile char stepsToPerpendicular = 0;														// Position of closest object (in half-steps) from current position
Object closestObject;

void scanIR(char steps, int motorDirection, char updateClosestObject);
void pointToObject();

/********** GLOBAL VARIABLES **********/

volatile bit RTC_FLAG_1MS = 0;
volatile bit RTC_FLAG_10MS = 0;
volatile bit RTC_FLAG_50MS = 0;
volatile bit RTC_FLAG_500MS = 0;
volatile bit goingHome, home = FALSE;
volatile bit ready = FALSE;
volatile bit rightWall, frontWall, leftWall = 0;

volatile char victimZone = 0;
volatile unsigned char xCoord = 1;
volatile unsigned char yCoord = 3;
volatile unsigned char node = 0;

volatile unsigned int RTC_Counter = 0;


PushButton start,eeprom;

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

		if(EEPROM_PB)																			// If PB1 has been pressed
		{
			eeprom.debounceCount++;															// Increment PB1 debounce count
			if(eeprom.debounceCount >= DEBOUNCE_REQ_COUNT & eeprom.released)						// If signal has been debounced sufficiently and switch has been released
			{
				eeprom.pressed = TRUE;															/* PB1 has been pressed       */
				eeprom.released = FALSE;														/* and therefore not released */
			}
		}
		else																				// If PB1 has not been pressed
		{
			eeprom.debounceCount = 0;															// Set PB1 debounce count to 0
			eeprom.released = TRUE;															// PB1 is therefore released
		}

		ser_int();
	}
}	

/******** INITIALISTIONS ********/

void init()
{
	start.pressed = FALSE;														/* Initialise all push buttons to not being pressed */
	start.released = TRUE;														/* but rather released                              */
	eeprom.pressed = FALSE;
	eeprom.released = TRUE;

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

void lookForVictim()
{
	ser_putch(142);
	ser_putch(17);
	char victim = ser_getch();
		
	if(victim > 241 && victim != 255)
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
		lcd_write_data('L');
	else
		lcd_write_data(' ');

	rotateIR(24, CW);
	frontWall = findWall();
	//Cliff
	if(xCoord == 2 && yCoord == 1)
	{
		frontWall = 1;
	}	
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

	rotateIR(36, CCW);	
}

void goParallel()
{
	PORTC |= 0b00000011;																	// Set CPLD to stepper motor module

	int distance, shortestDistance = 999;
	char stepsToWall;

	for (int step = -12; step <= 12; step++)
	{
		distance = readIR();
		if(distance < shortestDistance)
		{
			stepsToWall = step;
			shortestDistance = distance;
		}
		rotateIR(1, CCW);
	}
	rotateIR(12, CW);

	int angleParallelToWall = (int)((stepsToWall*HALF_STEP_RESOLUTION)-6); //6 is just for calibration, play with this value
	char angleHighByte = 0;
	char angleLowByte = (char) angleParallelToWall;
	
	if(angleParallelToWall < 0)																// If the angle is < 90
		angleParallelToWall = 360 + angleParallelToWall;									// Find the reflex angle

	if(angleParallelToWall > 255)															// If the angle is > 255
	{
		angleHighByte = 1;																	/* Split it into high */
		angleLowByte = (char)(angleParallelToWall - 255);									/* and low bytes      */
	}
	if((angleParallelToWall > 8) && (angleParallelToWall < 352))							// Only corrects if its out by more than 8deg (~1 step)
	{
		TURN_LEFT();																			/* Turn CCW on the spot   */
		waitFor(ANGLE,angleHighByte,angleLowByte);												/* To go parallel to wall */
		STOP();
	}
}

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
	lcd_write_string("(-,-) - -- --- -"); //x/y of robot, explore/return, zone of victim/got victim, walls, way went
	lcd_set_cursor(0x40);
	lcd_write_string("- - - (3,1) GREG"); //orientation, cliff detected, v.wall detected, x/y of final destination

	while(!home)
	{
		//Send accumulated data in EEPROM through serial
		if(eeprom.pressed && ready == FALSE)
		{
			EEPROMToSerial();
			eeprom.pressed = FALSE;
		}

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
			rotateIR(12, CCW);										
		}	
		
		if(start.pressed && ready == TRUE)
		{
			checkForFinalDestination();
			play_iCreate_song(5);
			lookForVictim();
			play_iCreate_song(5);
			findWalls();
			play_iCreate_song(5);
			if(leftWall)
				goParallel();
			else
				rotateIR(12, CCW);
			play_iCreate_song(5);
			if(frontWall)
				frontWallCorrect();
			play_iCreate_song(5);
			switch(node)
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
				updateMapData(0,0,0,0,0,getOrientation());
				updateLocation();
				updateNode();		
				if(goingHome)
					checkIfHome();
				play_iCreate_song(5);
			}
		}
	}
	//Turn off the create!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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