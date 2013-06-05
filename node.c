/*
 *  node.c
 *  
 *
 *  Created by Bryan Moutrie on 27/05/13.
 *  Copyright 2013 __MyCompanyName__. All rights reserved.
 *
 */
//#define BUILD
#if defined(BUILD)

#include "node.h"
#include "main.h"
#include "drive.h"

void node1()
{
	if (getGoingHome2()) //Write this function
	{
		if (getVictimZone2() == 1) // Write this function
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
	if (getGoingHome2()) //Write this function
	{
		if (getVictimZone2() == 2) // Write this function
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
	if (getGoingHome2()) //Write this function
	{
		if (getVictimZone2() == 3) // Write this function
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