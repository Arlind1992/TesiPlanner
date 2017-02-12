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

#include "grid/Grid.h"
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace rrt_planning
{
/*
 * constructor for testing purpose when the map is just a matrix
 */
Grid::Grid(Map& map,int maxX,int maxY):map(map),gridResolution(0),maxX(maxX),maxY(maxY){}

Grid::Grid(Map& map, double gridResolution): map(map),
    gridResolution(gridResolution)
{
    Bounds bounds = map.getBounds();

    maxX = floor((bounds.maxX - bounds.minX) / gridResolution);
    maxY = floor((bounds.maxY - bounds.minY) / gridResolution);
}


vector<Cell> Grid::getNeighbors(const Cell& s)
{
    int X = s.first;
    int Y = s.second;

    vector<Cell> neighbors;

    //Given (X,Y), retrive all the eight-connected free cells
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX-1 || Y+j > maxY-1)
                continue;
            Cell pos;
            pos.first= X+i;
            pos.second=Y+j;
            if(map.isFree(pos))
                neighbors.push_back(make_pair(X+i, Y+j));
        }

    return neighbors;
}

std::vector<Cell> Grid::getObstacles(const Cell& s)
{
    int X = s.first;
    int Y = s.second;

    vector<Cell> obstacles;

    //Given (X,Y), retrive all the eight-connected free cells
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX || Y+j > maxY)
                continue;

            Cell pos;
			pos.first= X+i;
            pos.second=Y+j;
            if(!map.isFree(pos))
                obstacles.push_back(make_pair(X+i, Y+j));
        }

    return obstacles;
}


double Grid::cost(const Cell& s, const Cell& s_next)
{
    //TODO no obstacles (8-connected)?

    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


double Grid::heuristic(const Cell& s, const Cell& s_next)
{
    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


bool Grid::lineOfSight(const Cell& s, const Cell& s_next)
{
    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    //Determine how steep the line is
    bool is_steep = abs(Y2-Y1) > abs(X2-X1);

    //Possibly rotate the line
    if(is_steep)
    {
        swap(X1, Y1);
        swap(X2, Y2);
    }

    if(X1 > X2)
    {
        swap(X1, X2);
        swap(Y1, Y2);
    }

    int error = (X2 - X1) / 2;
    int ystep = Y1 < Y2 ? 1 : -1;

    int y = Y1;

    //Check for obstalces through the line
    for(int x = X1; x <= X2; x++)
    {
        Cell pos;

        if(is_steep){
            pos.first = y;
            pos.second=x;
        }
        else{
            pos.first = x;
            pos.second=y;
        }
        if(!map.isFree(pos)) return false;

        error -= abs(Y2 - Y1);
        if(error < 0)
        {
            y += ystep;
            error += (X2 - X1);
        }
    }

    return true;
}


bool Grid::isFree(const Cell& s)
{
    return map.isFree(s);
}
std::vector<Cell> Grid::commCells(const Cell& s,int radius,const Cell& cgoal){

	int X = s.first;
	int Y = s.second;

	vector<Cell> comm;
	int starti,startj,endi,endj;
	if(X-radius<0){
		starti=0;
	}else{
		starti=X-radius;
	}
	if(Y-radius<0){
			startj=0;
		}else{
			startj=Y-radius;
	}
	if(X+radius>=this->maxX){
			endi=this->maxX-1;
		}else{
			endi=X+radius;
	}
	if(Y+radius>=this->maxY){
			endj=this->maxY-1;
		}else{
			endj=Y+radius;
	}

	//Given (X,Y), retrive all the eight-connected free cells
	for(int i = starti; i <= endi; i++)
	    for(int j = startj; j <= endj; j++)
	    {
	    	if(!(s.first==i&&s.second==j)){
	    	rrt_planning::Cell cell(i,j);
	    	if((map.isComm(cell)||((cgoal.first==cell.first)&&(cgoal.second==cell.second)))&&(heuristic(cell,s)<=radius)){
	        	if(map.isFree(cell)){
	        	comm.push_back(cell);
	        	}
	        	}
	    	}
	    }
	    return comm;
}

double Grid::pathCost(vector<Cell> path){
	double result=0;
	if(path.size()==0){
		return 0;
	}
	for (unsigned int it = 0 ; it < path.size()-1; it++){
		result=result+heuristic(path[it],path[it+1]);
	}
	return result;
}

bool Grid::isComm(const Cell& s){
	return this->map.isComm(s);
}
int Grid::getSpeed(const Cell& s){
	return this->map.getSpeed(s);
}

std::vector<Cell> Grid::getCommCells(const Cell& s,int radius){
	int X = s.first;
		int Y = s.second;

		vector<Cell> comm;
		int starti,startj,endi,endj;
		if(X-radius<0){
			starti=0;
		}else{
			starti=X-radius;
		}
		if(Y-radius<0){
				startj=0;
			}else{
				startj=Y-radius;
		}
		if(X+radius>=this->maxX){
				endi=this->maxX-1;
			}else{
				endi=X+radius;
		}
		if(Y+radius>=this->maxY){
				endj=this->maxY-1;
			}else{
				endj=Y+radius;
		}

		//Given (X,Y), retrive all the eight-connected free cells
		for(int i = starti; i <= endi; i++)
		    for(int j = startj; j <= endj; j++)
		    {
		    	if(!(s.first==i&&s.second==j)){
		    	rrt_planning::Cell cell(i,j);
		    	if(map.isComm(cell)&&(heuristic(cell,s)<=radius)){
		        	if(map.isFree(cell)){
		        	comm.push_back(cell);
		        		}
		        	}
		    	}
		    }
		    return comm;
}
int Grid::getMaxX(){
	return maxX;
}
int Grid::getMaxY(){
	return maxY;
}


rrt_planning::Cell Grid::fasterNeighbor(rrt_planning::Cell c){
	rrt_planning::Cell toReturn=c;
	for(int i=c.first-1;i<=c.first+1;i++){
		for(int j=c.second-1;j<=c.second+1;j++){
			if(i<0||j<0||i>(maxX-1)||j>(maxY-1))
							continue;
			int speedIJ=map.getSpeed(std::make_pair(i,j));
			int speedToReturn=map.getSpeed(std::make_pair(toReturn.first,toReturn.second));
			if(speedIJ>speedToReturn){
				toReturn=std::make_pair(i,j);
			}
		}
	}
	return toReturn;
}

rrt_planning::Cell Grid::getAntennaCenter(rrt_planning::Cell c){
	rrt_planning::Cell center=fasterNeighbor(c);
	if(center.first==c.first&&center.second==c.second){
		return center;
	}else{
		return getAntennaCenter(center);
	}
}
bool Grid::sameAntenna(Cell one,Cell two){
	Cell aCentOne=getAntennaCenter(one);
	Cell aCentTwo=getAntennaCenter(two);
	if(aCentOne.first==aCentTwo.first&&aCentOne.second==aCentTwo.second){
		return true;
	}else{
		return false;
	}

}

std::vector<Cell> Grid::getFourNeighbours(const Cell& s){
	std::vector<Cell> toReturn;
	for(int i=s.first-1;i<=s.first+1;i++){
		if(i==s.first)
			continue;
		if(i<0||i>=this->maxX)
			continue;
		if(this->isFree(std::make_pair(i,s.second)))
			toReturn.push_back(std::make_pair(i,s.second));
	}
	for(int i=s.second-1;i<=s.second+1;i++){
			if(i==s.second)
				continue;
			if(i<0||i>=this->maxY)
				continue;
			if(this->isFree(std::make_pair(s.first,i)))
				toReturn.push_back(std::make_pair(s.first,i));
		}
	return toReturn;

}





}
