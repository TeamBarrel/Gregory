#ifndef MAP_H
#define MAP_H

#include "drive.h"

void findFinalDestination(char virtualWallX, char virtualWallY, orientation robotOrientation);
char getFinalX();
char getFinalY();
char getVictimZone(char victimX, char victimY);

#endif