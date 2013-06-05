#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "ser.h"
#include "drive.h"
#include "xtal.h"
#include "sensors.h"

volatile bit moving = 0;

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
//	waitFor(DISTANCE,0x03,0xE8);	
//	STOP();
	while(moving)
	{
//		//checkSensors();
//		__delay_ms(50);
		ser_putch(142);
		ser_putch(19);
		high = ser_getch();
		low = ser_getch();
		deltaDistance = high*256 + low;
		distance += deltaDistance;
//		__delay_ms(50);
		if(distance >= moveDistance)
		{
			STOP();
			moving = 0;
		}
	}
}

bit isMoving()
{
	return moving;
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

void waitFor(char type, char highByte, char lowByte)
{
	__delay_ms(100);
	ser_putch(type);																		// Wait for angle of
	ser_putch(highByte);																		/* -90 */
	ser_putch(lowByte);																		/* deg */
}

#endif