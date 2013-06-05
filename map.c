#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "map.h"
#include "lcd.h"

char finalX = 0;
char finalY = 0;
char vicZone = 0;

void findFinalDestination(char virtualWallX, char virtualWallY, orientation robotOrientation)
{
	switch (virtualWallX)
	{
		case 0:
			switch (virtualWallY)
		{
				//case 0:  Default/initialised case
				//	break;
			case 1:
				finalX = 0;
				finalY = 1;
				break;
			case 2:
				finalX = 0;
				finalY = 2;					
				break;
			case 3:
				finalX = 0;
				finalY = 3;					
				break;
			default:
				break;
		}
			break;
			
		case 1:
			switch (virtualWallY)
		{
			case 0:
				finalX = 1;
				finalY = 0;					
				break;
			case 1:
				finalX = 1;
				finalY = 1;
				break;
			case 2:
				finalX = 1;
				finalY = 2;					
				break;
			case 3:
				finalX = 1;
				finalY = 3;					
				break;
			default:
				break;
		}
			break;
			
		case 2:
			switch (virtualWallY)
		{
			case 0:
				if(robotOrientation == WEST)
				{
					finalX = 3;
					finalY = 1;
				}					
				break;
			case 1:
				finalX = 2;
				finalY = 1;
				break;
			case 2:
				if(robotOrientation == EAST)
				{
					finalX = 2;
					finalY = 2;
				}
				break;
				//case 3:  Dead end			
				//	break;
			default:
				break;
		}
			break;
			
		case 3:
			switch (virtualWallY)
		{
			case 0:
				finalX = 3;
				finalY = 0;					
				break;
				//case 1:  Dead end
				//	break;
			case 2:
				finalX = 3;
				finalY = 2;					
				break;
				//case 3:  Dead end				
				//	break;
			default:
				break;
		}
			break;
			
		case 4:
			switch (virtualWallY)
		{
			case 0:
				finalX = 4;
				finalY = 0;					
				break;
			case 1:
				finalX = 4;
				finalY = 1;
				break;
			case 2:
				if (robotOrientation == SOUTH)
				{
					finalX = 4;
					finalY = 2;
				}
				break;
			case 3:
				finalX = 0;
				finalY = 0;					
				break;
			default:
				break;
		}
			break;
			
		default:
			break;
	}

	lcd_set_cursor(0x47);
	lcd_write_1_digit_bcd(finalX);
	lcd_set_cursor(0x49);
	lcd_write_1_digit_bcd(finalY);
}

char getFinalX()
{
	return finalX;
}

char getFinalY()
{
	return finalY;
}

char getVictimZone(char victimX, char victimY)
{
// ZONE MAP
// 00122
// 00000
// 44330
// 44000
	switch (victimX)
	{
		case 0:
			switch (victimY)
		{
			case 0:
				vicZone = 4;		
				break;
			case 1:
				vicZone = 4;
				break;
			//case 2:  Default/initialised case  					
			//	break;
			//case 3:  Default/initialised case  					
			//	break;
			default:
				break;
		}
			break;
			
		case 1:
			switch (victimY)
		{
			case 0:
				vicZone = 4;					
				break;
			case 1:
				vicZone = 4;
				break;
			//case 2:  Default/initialised case  					
			//	break;
			//case 3:  Default/initialised case  					
			//	break;
			default:
				break;
		}
			break;
			
		case 2:
			switch (victimY)
		{
			//case 0:  Default/initialised case  					
			//	break;
			case 1:
				vicZone = 3;
				break;
			//case 2:  Default/initialised case  					
			//	break;
			case 3:
				vicZone = 1;			
				break;
			default:
				break;
		}
			break;
			
		case 3:
			switch (victimY)
		{
			//case 0:  Default/initialised case  					
			//	break;
			case 1:
				vicZone = 3;
				break;
			//case 2:  Default/initialised case  					
			//	break;
			case 3:
				vicZone = 2;				
				break;
			default:
				break;
		}
			break;
			
		case 4:
			switch (victimY)
		{
			//case 0:  Default/initialised case  					
			//	break;
			//case 1:  Default/initialised case  					
			//	break;
			//case 2:  Default/initialised case  					
			//	break;
			case 3:
				vicZone = 2;					
				break;
			default:
				break;
		}
			break;
			
		default:
			break;
	}

	return vicZone;
}



#endif