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
    bool isComm(const Cell& s);
    int getSpeed(const Cell& s);
    std::vector<Cell> getCommCells(const Cell& s,int radious);
    std::vector<Cell> commCells(const Cell& s,int radious,const Cell& cgoal);
    double pathCost(std::vector<Cell> path);
    int getMaxX();
    int getMaxY();
    bool sameAntenna(Cell one,Cell two);
    std::vector<Cell> getFourNeighbours(const Cell& s);
    bool areFourConnected(Cell s1,Cell s2);

private:
    Map& map;

    double gridResolution;	// Cell edges in meters
    int maxX;
    int maxY;

	rrt_planning::Cell getAntennaCenter(rrt_planning::Cell c);
	rrt_planning::Cell fasterNeighbor(rrt_planning::Cell c);
};

}

#endif /* INCLUDE_RRT_PLANNING_GRID_GRID_H_ */
