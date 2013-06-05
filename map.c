//#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "map.h"
#include "main.h"

//VCMMNESW

char cellData[5][4];

void initCellData()
{
	for(int i = 0; i < 5; ++i)
	{
		for(int j = 0; j < 4; ++j)
		{
			cellData[i][j] = 0;
		}
	}
}

void writeCellData()
{
	cellData[xCoord][yCoord] = (orientation << 3);
	
	if (walls[WEST])
		cellData[xCoord][yCoord] |= 0b00000001;
	if (walls[SOUTH])
		cellData[xCoord][yCoord] |= 0b00000010;
	if (walls[EAST])
		cellData[xCoord][yCoord] |= 0b00000100;
	if (walls[NORTH])
		cellData[xCoord][yCoord] |= 0b00001000;
	
}

#endif