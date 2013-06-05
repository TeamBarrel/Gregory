#ifndef DRIVE_H
#define DRIVE_H

#include <htc.h>

#define TIME 155
#define DISTANCE 156
#define ANGLE 157
#define EVENT 157

#define DRIVE_STRAIGHT() drive(0, 125, 128, 0)												// +125mm/s 											// -100mm/s
#define STOP() drive(0, 0, 0, 0)
#define TURN_RIGHT() drive(0, 25, 255, 255)
#define TURN_LEFT() drive(0, 25, 0, 1)

void drive(char highByteSpeed, char lowByteSpeed, char highByteRadius, char lowByteRadius);
void driveForDistance(int moveDistance);
void turnAround();
void turnLeft90();
void turnRight90();
void waitFor(char type, char highByte, char lowByte);

#endif