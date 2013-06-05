#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "sensors.h"
#include "ser.h"
#include "drive.h"

char sensorData = 0;
bit sensorDetected = 0;

bit detectCliff()
{
	sensorDetected = 0;
	ser_putch(149);
	ser_putch(4);
	ser_putch(9);
	ser_putch(10);
	ser_putch(11);
	ser_putch(12);
		
	sensorData = ser_getch();
	if(sensorData == 1)
		sensorDetected = 1;

	sensorData = ser_getch();
	if(sensorData == 1 && !sensorDetected)
		sensorDetected = 1;

	sensorData = ser_getch();
	if(sensorData == 1 && !sensorDetected)
		sensorDetected = 1;

	sensorData = ser_getch();
	if(sensorData == 1 && !sensorDetected)
		sensorDetected = 1;
	
	return sensorDetected;
}

// Return the bit at a position of a char
bit getBit(char byte, int position)
{
   return (byte >> position) & 1;
}

#endif