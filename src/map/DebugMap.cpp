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

#include "map/DebugMap.h"
namespace rrt_planning
{

DebugMap::DebugMap(MatrixDyn* mat,CommGrid* comm)
{
repMatrix=mat;
this->comm=comm;

}

bool DebugMap::isFree(const rrt_planning::Cell& s)
{
    if((*repMatrix)(s.first,s.second)==0)
    	return true;
    else
    	return false;
}
bool DebugMap::isComm(const rrt_planning::Cell& s){
	int speed;
	if(this->comm->getSpeed(s.first,s.second,speed)){
		return true;
	}else{
		return false;
	}
}


DebugMap::~DebugMap()
{

}

}
