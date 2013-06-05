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
	volatile char high, low;
	int deltaDistance = 0;	
	int distance = 0;

	moving = 1;
	DRIVE_STRAIGHT();

	while(moving)
	{
		ser_putch(142);
		ser_putch(19);
		high = ser_getch();
		low = ser_getch();
		deltaDistance = high*256 + low;
		distance += deltaDistance;

//		if(cliff is detected)
//		{
//			STOP();
//			goReverse();
//			signal not to try again
//			break;
//		}

//		if(virtual wall is detected)
//		{
//			STOP();
//			findFinalDestination(getCurrentX(),getCurrentY(), currentOrientation);
//			goReverse();
//			signal not to try again
//			break;
//		}

		if(distance >= moveDistance)
			moving = FALSE;
	}
	STOP();
}

orientation getOrientation()
{
	return currentOrientation;
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
	driveForDistance(1000);
	lastMove = FORWARD;
}

// Go one cell left
void goLeft()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('L');
	turnLeft90();
	updateOrientation(LEFT);
	driveForDistance(1000);
	lastMove = LEFT;
}

void goReverse()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('!');
	REVERSE();
	waitFor(DISTANCE,1,244);
	STOP();
	if(lastMove == LEFT)
	{
		lcd_set_cursor(0x0F);
		lcd_write_data('R');
		turnRight90();
		updateOrientation(RIGHT);
	}
	else if (lastMove == RIGHT)
	{
		lcd_set_cursor(0x0F);
		lcd_write_data('L');
		turnLeft90();
		updateOrientation(LEFT);
	}
}

// Go one cell right
void goRight()
{
	lcd_set_cursor(0x0F);
	lcd_write_data('R');
	turnRight90();
	updateOrientation(RIGHT);
	driveForDistance(1000);
	lastMove = RIGHT;
}

void turnAround()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,170); 
	STOP();
	__delay_ms(6500);
	__delay_ms(6500);
}

void turnLeft90()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,85); 
	STOP();
	__delay_ms(6500);
}

void turnRight90()
{
	TURN_RIGHT();																		// Turn CW on the spot
	waitFor(ANGLE,255,169); 
	STOP();
	__delay_ms(6500);
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

void rightWallCorrect(void)
{
	turnRight90();
	rotateIR(24, CCW);
	while(readIR() <45)
	{
		REVERSE();
	}
		while(readIR() >55)
	{
		DRIVE_STRAIGHT();
	}
	turnLeft90();
	rotateIR(24, CW);
	STOP();
}

void frontWallCorrect(void)
{
	while(readIR() < 50)
		{
			REVERSE();
		}
	STOP();
	
	while(readIR() > 55 && readIR() < 100)
		{
			DRIVE_STRAIGHT();
		}
	STOP();
}	

#endif