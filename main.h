#ifndef	MAIN_H
#define MAIN_H


/********** #DEFINES **********/

#define TRUE 1
#define FALSE 0
#define TMR0_VAL 100																		// TMR0 settings: Start count, period of 1ms, with prescaler = 32
#define DEBOUNCE_REQ_COUNT 10
#define START_PB !RB0
#define CW 0b00001111
#define CCW 0b00001101

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

// A push button on the DSX board
typedef struct {
	char pressed;																			// Denotes if the push button is pressed
	char released;																			// Denotes if the push button is released
	char debounceCount;																		// Denotes the number of high signals from buton when debouncing
} PushButton;

/********** PROTOTYPES **********/

void init();
void initIRobot();
bit findWall();
void findWalls();
//bit getBit(char byte, int position);
void goBackward();
void goForward();
void goLeft();
void goToNextCell();
void goRight();
void run();
void updateLocation();
void updateOrientation();

#endif MAIN_H 