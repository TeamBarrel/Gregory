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
#include "songs.h"

volatile bit successfulDrive = 0;
volatile direction somethingInTheWay = BACKWARD;
volatile bit moving = FALSE;
volatile direction wayWent;
volatile direction lastMove;
volatile orientation currentOrientation = WEST;

// Basic drive function for the Create
void drive(char highByteSpeed, char lowByteSpeed, char highByteRadius, char lowByteRadius)
{
	__delay_ms(100);
	ser_putch(137);
	ser_putch(highByteSpeed);
	ser_putch(lowByteSpeed);
	ser_putch(highByteRadius);
	ser_putch(lowByteRadius);
}

// Drives the Create forward for a specified distance, unless a cliff or virtual wall is detected
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
			if(cliff != 0)
			{
				STOP();
				goReverse();
				clearSuccessfulDrive();
	
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
			setVirtualLocation(getCurrentX(), getCurrentY(), currentOrientation);
			goReverse();
			clearSuccessfulDrive();
			
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
			somethingInTheWay = BACKWARD; // Nothing in the way
		}
	}	
}

// Returns the orientation of the Create
orientation getOrientation()
{
	return currentOrientation;
}

// Returns the direction of a cliff or virtual wall
direction getSomethingInTheWay()
{
	return somethingInTheWay;
}

// Returns whether the drive was successful (no virtual walls or cliffs encountered)
bit getSuccessfulDrive()
{
	return successfulDrive;
}

// Returns the direction the Create moved
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

// Reverses 30cm to back away from a cliff / virtual wall / wall
void goReverse()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('!');
	REVERSE();	
	waitFor(DISTANCE,254,212);
	STOP();
	__delay_ms(2000);
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

// Turns around 180def
void turnAround()
{
	TURN_LEFT();
	waitFor(ANGLE,0,180); 
	STOP();
	__delay_ms(3000);				//Makes DSX wait until its stopped
	__delay_ms(3000);				//Makes DSX wait until its stopped
}

// Turns left 90deg on the spot
void turnLeft90()
{
	TURN_LEFT();
	waitFor(ANGLE,0,90); 
	STOP();
	__delay_ms(3000);				//Makes DSX wait until its stopped
}

// Turns right 90deg on the spot
void turnRight90()
{
	TURN_RIGHT();
	waitFor(ANGLE,255,174); 
	STOP();
	__delay_ms(3000);				//Makes DSX wait until its stopped
}

// Updates the orientation - uses enumerated types
void updateOrientation(direction moved)
{
	currentOrientation += moved;
	if(currentOrientation >= 4)
		currentOrientation -= 4;
}

// Sends a wait command to the Create
void waitFor(char type, char highByte, char lowByte)
{
	__delay_ms(100);
	ser_putch(type);
	ser_putch(highByte);
	ser_putch(lowByte);
}


// Corrects the distance away from a front wall
void frontWallCorrect(void)
{

	int distToWall = readIR();		//find distance
	if(distToWall < 45)				//correct only if its more than 5cm out
	{
		REVERSE();					//called only once now
		while(distToWall < 51)		//Wait until its 50cm away
			distToWall = readIR(); 
		STOP();
		clearSuccessfulDrive();
	}
	else if(distToWall > 55) //I took the < 100 part out, the code only executes if there is a front wall already
	{
		DRIVE_STRAIGHT();			//called only once now
		while(distToWall > 49)		//Wait until its 50cm away
			distToWall = readIR();
		STOP();
		clearSuccessfulDrive();
	}

}	

// Turns the create 8deg back towards the centre of the cell if it is more than 14cm away from the middle
void leftAngleCorrect()
{
	int distanceToWall = readIR();
	if((distanceToWall > 86) && (distanceToWall < 100))
	{
		TURN_LEFT();
		waitFor(ANGLE,0,8);
		STOP();
		__delay_ms(1000);
	}
	else if(distanceToWall < 36)
	{

		TURN_RIGHT();
		waitFor(ANGLE,255,0b11111000);
		STOP();
		__delay_ms(1000);
	}
}


#endif