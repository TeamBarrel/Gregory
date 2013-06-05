/*
 *  node.c
 *  
 *
 *  Created by Bryan Moutrie on 27/05/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */
#define BUILD
#if define(BUILD)

#include "node.h"
#include "main.h"
#include "drive.h"

void node1()
{
	if (getGoingHome()) //Write this function
	{
		if (getVictimCell() == 1) // Write this function
			goRight();
		else if (getOrientation() == EAST)
			goForward();
		else if (getOrientation() == SOUTH)
			goRight();
	}
	else
		goToNextCell();
}

void node2()
{
	if (getGoingHome()) //Write this function
	{
		if (getVictimCell() == 2) // Write this function
			goForward();
		else if (getOrientation() == SOUTH)
			goRight();
		else if (getOrientation() == NORTH)
			goLeft();
	}
	else
		goToNextCell();
}

void node3()
{
	if (getGoingHome()) //Write this function
	{
		if (getVictimCell() == 3) // Write this function
			goRight();
		else if (getOrientation() == EAST)
			goForward();
		else if (getOrientation() == SOUTH)
			goLeft();
	}
	else
		goToNextCell();
}


#endif