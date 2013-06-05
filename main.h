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



// A push button on the DSX board
typedef struct {
	char pressed;																			// Denotes if the push button is pressed
	char released;																			// Denotes if the push button is released
	char debounceCount;																		// Denotes the number of high signals from buton when debouncing
} PushButton;

/********** PROTOTYPES **********/

void init();
void initIRobot();
void checkForFinalDestination();
void lookForVictim();
void findWalls();
void goToNextCell();
//void sendEEPROMData();
void updateLocation();
void updateNode();
void checkIfHome();

char getCurrentX();
char getCurrentY();

#endif MAIN_H 