/*
 * AbstractPlanner.h
 *
 *  Created on: Feb 12, 2017
 *      Author: arlind
 */

#ifndef PLANNERS_ABSTRACTPLANNER_H_
#define PLANNERS_ABSTRACTPLANNER_H_
#include "grid/Cell.h"
using namespace rrt_planning;
namespace planner {

class AbstractPlanner {
public:
	virtual bool makePlan(Cell start,Cell end,std::vector<Cell>& path,int buffer)=0;
	virtual ~AbstractPlanner(){

	}
};

} /* namespace planner */

#endif /* PLANNERS_ABSTRACTPLANNER_H_ */
