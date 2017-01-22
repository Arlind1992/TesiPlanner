/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Alessandro Riva
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_RRT_PLANNING_GRID_GRID_H_
#define INCLUDE_RRT_PLANNING_GRID_GRID_H_

#include <vector>

#include "map/Map.h"
#include "grid/Cell.h"

namespace rrt_planning
{

class Grid
{

public:
	Grid(Map& map,int maxX,int maxY);
    Grid(Map& map, double gridResolution);
    double cost(const Cell& s, const Cell& s_next);
    double heuristic(const Cell& s, const Cell& s_next);
    bool lineOfSight(const Cell& s, const Cell& s_next);
    std::vector<Cell> getNeighbors(const Cell& s);
    std::vector<Cell> getObstacles(const Cell& s);
    bool isFree(const Cell& s);
    std::vector<Cell> commCells(const Cell& s,int radious,const Cell& cgoal);
    //TODO implement
    double pathCost(std::vector<Cell> path);

private:
    Map& map;

    double gridResolution;	// Cell edges in meters
    int maxX;
    int maxY;
};

}

#endif /* INCLUDE_RRT_PLANNING_GRID_GRID_H_ */
