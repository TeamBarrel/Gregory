//#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "sensors.h"
#include "ser.h"
#include "xtal.h"
#include "drive.h"

volatile unsigned char check;
volatile bit cliffDetected = 0;
volatile bit virtualWallDetected = 0;

void checkCliff()
{
		__delay_ms(100);
		ser_putch(149);
		ser_putch(5);
		ser_putch(7);
		ser_putch(9);
		ser_putch(10);
		ser_putch(11);
		ser_putch(12);
		
		//Bump Sensors
		check = ser_getch();
		if(getBit(check,0) || getBit(check,1)){
			STOP();
		}
		//Cliff Sensors
		check = ser_getch();
		if(check == 1){
			STOP();
		}
		check = ser_getch();
		if(check == 1){
			STOP();
		}
		check = ser_getch();
		if(check == 1){
			STOP();
		}
		check = ser_getch();
		if(check == 1){
			STOP();
		}

/*	cliffDetected = 0;

	ser_putch(149);
	ser_putch(4);
	ser_putch(9);
	ser_putch(10);
	ser_putch(11);
	ser_putch(12);
	
	//Cliff Sensors
	check = ser_getch();
	if(check == 1)
	{
		STOP();
		cliffDetected = 1;
	}
	check = ser_getch();
	if(check == 1)// && !cliffDetected)
	{
		STOP();
		cliffDetected = 1;
	}
	check = ser_getch();
	if(check == 1)// && !cliffDetected)
	{
		STOP();
		cliffDetected = 1;
	}
	check = ser_getch();
	if(check == 1)// && !cliffDetected)
	{
		STOP();
		cliffDetected = 1;
	}

	return cliffDetected;
*/
}

bit checkVirtualWall()
{
	virtualWallDetected = 0;

	ser_putch(149);
	ser_putch(1);
	ser_putch(13);

	check = ser_getch();
	if(check == 1)
	{
		STOP();
		virtualWallDetected = 1;
	}

	return virtualWallDetected;
}

bit checkVictim()
{
	ser_putch(149);
	ser_putch(1);
	ser_putch(17);

	if(ser_getch() == 254)
		return 1;
	else
		return 0;
}

//void checkSensors()
//{
//
////	while(!done)
////	{
////		__delay_ms(100);
//		ser_putch(149);
//		ser_putch(5);
//		ser_putch(7);
//		ser_putch(9);
//		ser_putch(10);
//		ser_putch(11);
//		ser_putch(12);
//		
//		//Bump Sensors
//		check = ser_getch();
//		if(getBit(check,0) || getBit(check,1)){
//			STOP();
////			done = 1;
//		}
//		//Cliff Sensors
//		check = ser_getch();
//		if(check == 1){
//			STOP();
////			done = 1;
//		}
//		check = ser_getch();
//		if(check == 1){
//			STOP();
////			done = 1;
//		}
//		check = ser_getch();
//		if(check == 1){
//			STOP();
////			done = 1;
//		}
//		check = ser_getch();
//		if(check == 1){
//			STOP();
////			done = 1;
//		}
//}

// Return the bit at a position of a char
bit getBit(char byte, int position)
{
   return (byte >> position) & 1;
}

#endif