//#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "sensors.h"
#include "ser.h"
//#include "main.h"
#include "drive.h"

bit check;

void checkSensors()
{

//	while(!done)
//	{
//		__delay_ms(100);
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
//			done = 1;
		}
		//Cliff Sensors
		check = ser_getch();
		if(check == 1){
			STOP();
//			done = 1;
		}
		check = ser_getch();
		if(check == 1){
			STOP();
//			done = 1;
		}
		check = ser_getch();
		if(check == 1){
			STOP();
//			done = 1;
		}
		check = ser_getch();
		if(check == 1){
			STOP();
//			done = 1;
		}
}

// Return the bit at a position of a char
bit getBit(char byte, int position)
{
   return (byte >> position) & 1;
}

#endif