#pragma once

#ifndef	INFLUENCEMAP_H
#define	INFLUENCEMAP_H

// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

#include "BZDBCache.h"

#define FACTOR		1/16

class InfluenceMap
{
// functions
public:
	InfluenceMap(int, int);
	int		getInfluence(int, int);
	void	update();

// variables
private:
	int		x;
	int		y;
	int**	influenceArray;
	int	matrix[3][3] = { {1, 2, 1},
						 {2, 4, 2},
						 {1, 2, 1} };
};

#endif // INFLUENCEMAP_H