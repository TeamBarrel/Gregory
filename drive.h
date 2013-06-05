#ifndef DRIVE_H
#define DRIVE_H

#include <htc.h>

/*********** TYPEDEFS ***********/

typedef enum {
	FORWARD = 0,
	LEFT = 1,
	BACKWARD = 2,
	RIGHT = 3
} direction;

typedef enum {
	WEST = 0,
	SOUTH = 1,
	EAST = 2,
	NORTH = 3
} orientation;

#define TIME 155
#define DISTANCE 156
#define ANGLE 157
#define EVENT 157

#define DRIVE_STRAIGHT() drive(0, 250, 128, 0)												// +125mm/s 											// -100mm/s
#define STOP() drive(0, 0, 0, 0)
#define TURN_RIGHT() drive(0, 50, 255, 255)
#define TURN_LEFT() drive(0, 50, 0, 1)
#define REVERSE() drive(255, 125, 128, 0)

void drive(char highByteSpeed, char lowByteSpeed, char highByteRadius, char lowByteRadius);
void driveForDistance(int moveDistance);
orientation getOrientation();
direction getSomethingInTheWay();
bit getSuccessfulDrive();
direction getWayWent();
void goBackward();
void goForward();
void goLeft();
void goReverse();
void goRight();
void turnAround();
void turnLeft90();
void turnRight90();
void updateOrientation(direction moved);
void waitFor(char type, char highByte, char lowByte);
void rightWallCorrect(void);
void frontWallCorrect(void);

#endif