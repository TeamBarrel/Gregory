#ifndef	SENSORS_H
#define SENSORS_H

#include <htc.h>

void checkCliff();
bit checkVirtualWall();
bit checkVictim();
bit getBit(char byte, int position);

#endif