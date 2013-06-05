#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "ser.h"
#include "drive.h"
#include "xtal.h"
#include "sensors.h"
#include "lcd.h"
#include "map.h"
#include "main.h"
#include "ir.h"

volatile bit successfulDrive = 0;
volatile direction somethingInTheWay = BACKWARD;
volatile bit moving = FALSE;
volatile direction wayWent;
volatile direction lastMove;
volatile orientation currentOrientation = WEST;

void drive(char highByteSpeed, char lowByteSpeed, char highByteRadius, char lowByteRadius)
{
	__delay_ms(100);
	ser_putch(137);
	ser_putch(highByteSpeed);
	ser_putch(lowByteSpeed);
	ser_putch(highByteRadius);
	ser_putch(lowByteRadius);
}

void driveForDistance(int moveDistance)
{
	//Distance Counter
	volatile char high, low, cliff, virtualWall;
	int deltaDistance = 0;	
	int distance = 0;

	moving = TRUE;
	DRIVE_STRAIGHT();
	successfulDrive = FALSE;
	
	while(moving)
	{
		if(distance >= 100)
		{
			//Cliff	
			ser_putch(142);
			ser_putch(10);
			cliff = ser_getch();
			if(cliff == 0)
			{
				ser_putch(142);
				ser_putch(11);
				cliff = ser_getch();
				if(cliff == 0)
				{
					ser_putch(142);
					ser_putch(9);
					cliff = ser_getch();
					if(cliff == 0)
					{
						ser_putch(142);
						ser_putch(12);
						cliff = ser_getch();
					}
				}
			}
			if(cliff == 1)
			{
				STOP();
				goReverse();
	
				if(lastMove == LEFT)
				{
					somethingInTheWay = LEFT;
					turnRight90();
					updateOrientation(RIGHT);
				}
				else if (lastMove == RIGHT)
				{
					somethingInTheWay = RIGHT;
					turnLeft90();
					updateOrientation(LEFT);
				}
				else
					somethingInTheWay = FORWARD;
				moving = FALSE;
			}	
		}

		// Virtual Wall
		ser_putch(142);
		ser_putch(13);
		virtualWall = ser_getch();
		if(virtualWall == 1)
		{
			STOP();
			findFinalDestination(getCurrentX(),getCurrentY(), currentOrientation);
			goReverse();

			if(lastMove == LEFT)
			{
				somethingInTheWay = LEFT;
				turnRight90();
				updateOrientation(RIGHT);
			}
			else if (lastMove == RIGHT)
			{
				somethingInTheWay = RIGHT;
				turnLeft90();
				updateOrientation(LEFT);
			}
			else
				somethingInTheWay = FORWARD;
			moving = FALSE;
		}

		// Distance
		ser_putch(142);
		ser_putch(19);
		high = ser_getch();
		low = ser_getch();
		deltaDistance = high*256 + low;
		distance += deltaDistance;	
		if(distance >= moveDistance)
		{
			STOP();
			successfulDrive = TRUE;
			moving = FALSE;
			somethingInTheWay = BACKWARD;
		}
	}	
}

orientation getOrientation()
{
	return currentOrientation;
}

direction getSomethingInTheWay()
{
	return somethingInTheWay;
}


bit getSuccessfulDrive()
{
	return successfulDrive;
}

direction getWayWent()
{
	return wayWent;
}

// Go one cell backwards
void goBackward()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('B');
	turnAround();
	updateOrientation(BACKWARD);
	driveForDistance(1000);
	lastMove = BACKWARD;
}

// Go one cell forwards
void goForward()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('F');
	lastMove = FORWARD;
	driveForDistance(1000);
}

// Go one cell left
void goLeft()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('L');
	turnLeft90();
	updateOrientation(LEFT);
	lastMove = LEFT;
	driveForDistance(1000);
}

void goReverse()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('!');
	REVERSE();	
	waitFor(DISTANCE,254,12);		//Twos complement of 50cm - change
	STOP();
	__delay_ms(2000);				//Makes DSX wait until its stopped - change is waitFor is changed
}

// Go one cell right
void goRight()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('R');
	turnRight90();
	updateOrientation(RIGHT);
	lastMove = RIGHT;
	driveForDistance(1000);
}

void turnAround()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,170); 
	STOP();
	__delay_ms(6000);				//Makes DSX wait until its stopped - change is waitFor is changed
	__delay_ms(6000);				//Makes DSX wait until its stopped - change is waitFor is changed
}

void turnLeft90()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,85); 
	STOP();
	__delay_ms(6000);				//Makes DSX wait until its stopped - change is waitFor is changed
}

void turnRight90()
{
	TURN_RIGHT();																		// Turn CW on the spot
	waitFor(ANGLE,255,169); 
	STOP();
	__delay_ms(6000);				//Makes DSX wait until its stopped - change is waitFor is changed
}

void updateOrientation(direction moved)
{
	currentOrientation += moved;
	if(currentOrientation >= 4)
		currentOrientation -= 4;
}

void waitFor(char type, char highByte, char lowByte)
{
	__delay_ms(100);
	ser_putch(type);																		// Wait for angle of
	ser_putch(highByte);																		/* -90 */
	ser_putch(lowByte);																		/* deg */
}

//void rightWallCorrect(void)
//{
//	turnRight90();
//	rotateIR(24, CCW);
//	while(readIR() <45)
//	{
//		REVERSE();
//	}
//		while(readIR() >55)
//	{
//		DRIVE_STRAIGHT();
//	}
//	turnLeft90();
//	rotateIR(24, CW);
//	STOP();
//}

void frontWallCorrect(void)
{
	rotateIR(24, CW);				//rotate IR from left to forward
	int distToWall = readIR();		//find distance
	if(distToWall < 45)				//correct only if its more than 5cm out
	{
		REVERSE();					//called only once now
		while(distToWall < 51)		//Wait until its 50cm away
			distToWall = readIR(); 
		STOP();
	}
	else if(distToWall > 55) //I took the < 100 part out, the code only executes if there is a front wall already
	{
		DRIVE_STRAIGHT();			//called only once now
		while(distToWall > 49)		//Wait until its 50cm away
			distToWall = readIR();
		STOP();
	}
	rotateIR(24, CCW);				//rotate IR from forward to left
}	

#endif