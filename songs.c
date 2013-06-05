#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "songs.h"

void ser_putArr(unsigned char array[], int length);

//VARIABLE

unsigned char superMarioBros[] = {140, 1, 11, 76, 8, 128, 4, 76, 16, 128, 4, 76, 16, 128, 4, 72, 8, 76, 16, 79, 32, 128, 8, 67, 16};
unsigned char lookingForU2[] = {140, 2, 13, 72, 28, 72, 28, 79, 64, 128, 16, 77, 28, 76, 28, 72, 64, 128, 16, 69, 28, 69, 28, 69, 28, 72, 28, 72, 64};
unsigned char finalCountdown[] = {140, 3, 12, 73, 8, 71, 8, 73, 32, 66, 64, 128, 32, 74, 8, 73, 8, 74, 8, 128, 12, 73, 6, 128, 12, 71, 48};
unsigned char champions[] = {140, 4, 9, 74, 64, 73, 16, 74, 16, 73, 48, 69, 48, 128, 32, 66, 28, 71, 32 ,66, 48};  
	
//FUNCTIONS

/* Implements a user defined array to play a song on the iRobot Create  */



/* Plays an inbuilt song on the iRobot Create  */

void play_iCreate_song(unsigned char song)
{
	ser_putch(141);
	ser_putch(song);
}

void initSongs()
{
	ser_putArr(finalCountdown, 27);
	ser_putArr(superMarioBros, 25);
	ser_putArr(lookingForU2, 29);
	ser_putArr(champions, 21);
}

#endif

