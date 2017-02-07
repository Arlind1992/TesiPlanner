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

#include "rrt_planning/ThetaStarPlanner.h"

/*TODO ros
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarPlanner, nav_core::BaseGlobalPlanner)
*/
using namespace std;
using namespace Eigen;

//Default Constructor
namespace rrt_planning
{

const Cell ThetaStarPlanner::S_NULL = make_pair(-1, -1);

ThetaStarPlanner::ThetaStarPlanner(Grid* grid):grid(grid){}
bool ThetaStarPlanner::makePlan(Cell start,Cell goal,std::vector<Cell>& path,int buffer)
{
    clearInstance();
    //Init the position of the special states
    s_start = start;
    s_goal = goal;

    //Test starting position
    if(!grid->isFree(s_start))
    {

        return false;
    }

    //Test target position
    if(!grid->isFree(s_goal))
    {

        return false;
    }

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
    open.insert(s_start, grid->heuristic(s_start, s_goal));
    parent[s_goal] = S_NULL;

    //Compute plan
    while(!open.empty())
    {
        //Pop the best frontier node
        Cell s = open.pop();

        closed.insert(s);

        if(s == s_goal) break;

        //TODO ask it should work
        if(grid->heuristic(s,s_goal)>buffer){
        	return false;
        }

        for(auto s_next: grid->getNeighbors(s))
        	if(closed.count(s_next) == 0)
            {
                if(!open.contains(s_next))
                {
                    g[s_next] = std::numeric_limits<double>::infinity();
                    parent[s_next] = S_NULL;
                }

                updateVertex(s, s_next);
            }
    }

    //Publish plan
    auto state = s_goal;
    path.push_back(state);
    do
    {
        state = parent.at(state);

        if(state == S_NULL)
        {
            return false;
        }

        path.push_back(state);
    }
    while(state != s_start);

    reverse(path.begin(), path.end());
    return true;
}


void ThetaStarPlanner::updateVertex(Cell s, Cell s_next)
{
    double g_old = g.at(s_next);

    computeCost(s, s_next);

    if(g.at(s_next) < g_old)
    {
        if(open.contains(s_next))
            open.remove(s_next);

        double frontierCost = g.at(s_next) + grid->heuristic(s_next, s_goal);

        open.insert(s_next, frontierCost);
    }
}


void ThetaStarPlanner::computeCost(Cell s, Cell s_next)
{

    if(grid->lineOfSight(parent.at(s), s_next))
    {
        //Path 2
        if(g.at(parent.at(s)) + grid->cost(parent.at(s), s_next) <= g.at(s_next))
        {
            parent.at(s_next) = parent.at(s);
            g.at(s_next) = g.at(parent.at(s)) + grid->cost(parent.at(s), s_next);
        }
    }
    else
    {
        //Path 1
        if(g.at(s) + grid->cost(s, s_next) <= g.at(s_next))
        {
            parent.at(s_next) = s;
            g.at(s_next) = g.at(s) + grid->cost(s, s_next);
        }
    }
}

void ThetaStarPlanner::clearInstance()
{
    open.clear();
    closed.clear();
    parent.clear();
    g.clear();
}



ThetaStarPlanner::~ThetaStarPlanner()
{
    if(grid)
        delete grid;
}


};
