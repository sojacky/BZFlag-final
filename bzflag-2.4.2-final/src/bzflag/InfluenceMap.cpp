#include "InfluenceMap.h"
#include <math.h>
#include "common.h"
#include "BZDBCache.h"
//#include "World.h"
//#include "playing.h" 


InfluenceMap::InfluenceMap(int x, int y) 
{
	this->x = x;
	this->y = y;

	this->influenceArray = new int*[x];
	for (int i = 0; i < x; ++i) {
		influenceArray[i] = new int[y];
	}
}



int InfluenceMap::getInfluence(int x, int y)
{
	return influenceArray[x][y];
}

void InfluenceMap::update()
{
	for (int i = 0; i < x; i++)	{
		for (int j = 0; j < y; j++) {
			for (int k = x-1; k < x+2; k++) {
				for (int l = y-1; l < y+2; l++) {
					if ((k < 0 || k >= x) || (l < 0 || l >= y)){

					}
					else {
						influenceArray[k][l] = influenceArray[k][l];
					}
				}
			}
		}
	}
}