/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#ifndef INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_
#define INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_


#include <eigen3/Eigen/Dense>

#include "grid/Grid.h"
#include "rrt_planning/theta/PriorityQueue.h"


namespace rrt_planning
{

class ThetaStarPlanner
{

public:
    ThetaStarPlanner(Grid* grid);
    //TODO add map
    ThetaStarPlanner(std::string name);

    void initialize(std::string name);
    bool makePlan(Cell start,Cell goal,std::vector<Cell>& path);

    ~ThetaStarPlanner();

private:
    void updateVertex(Cell s, Cell s_next);
    void computeCost(Cell s, Cell s_next);
    void clearInstance();
    void displayClosed();
    void displayOpen();
private:


    static const Cell S_NULL;

    Grid* grid;

    Cell s_start;
    Cell s_goal;

    std::map<Cell, double> g;
    PriorityQueue open;
    std::map<Cell, Cell> parent;
    std::set<Cell> closed;

 };

}

#endif /* INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_ */
