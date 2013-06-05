#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "ser.h"
#include "drive.h"
#include "xtal.h"

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
	
	DRIVE_STRAIGHT();
	
	while(distance >= moveDistance)
	{
		ser_putch(137);
		ser_putch(19);
		high = ser_getch();
		low = ser_getch();
		deltaDistance = high*256 + low;
		distance += deltaDistance;
	}
	STOP();
}

void turnAround()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,170); 
	STOP();
}

void turnLeft90()
{
	TURN_LEFT();																		// Turn CW on the spot
	waitFor(ANGLE,0,85); 
	STOP();
}

void turnRight90()
{
	TURN_RIGHT();																		// Turn CW on the spot
	waitFor(ANGLE,255,169); 
	STOP();
}

void waitFor(char type, char highByte, char lowByte)
{
	__delay_ms(100);
	ser_putch(type);																		// Wait for angle of
	ser_putch(highByte);																		/* -90 */
	ser_putch(lowByte);																		/* deg */
}

#endif