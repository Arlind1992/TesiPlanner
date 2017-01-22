/*
 * ComplexPlanner.h
 *
 *  Created on: Jan 21, 2017
 *      Author: arlind
 */


#ifndef INCLUDE_COMPLEXPLANNER_H_
#define INCLUDE_COMPLEXPLANNER_H_

#include "Planner.h"
#include "grid/Grid.h"
#include "grid/Cell.h"
#include "rrt_planning/ThetaStarPlanner.h"
#include <lemon/list_graph.h>

namespace planner {

class ComplexPlanner{
public:
	ComplexPlanner(rrt_planning::Grid* grid,int buffer):grid(grid),planner(grid),length(graph),buffer(0){

	}
	virtual ~ComplexPlanner();
	 bool makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result);


private:

		/*
		 * Attributes
		 */
		rrt_planning::Grid* grid;
	    //graph that is going to be used to find the optimal path
	    lemon::ListGraph graph;
	    rrt_planning::ThetaStarPlanner planner;
	    Graph::EdgeMap<double> length;
	    std::map<Graph::Arc,std::vector<rrt_planning::Cell> > edgePath;
	    std::map<Graph::Node,rrt_planning::Cell> nodeCell;
	    //TODO probably don't need it so delete
	    std::map<rrt_planning::Cell,Graph::Node> cellNode;
	    int buffer;

	    /*
	     * helper functions
	     */
	     bool searchCell(std::set<rrt_planning::Cell> cells,rrt_planning::Cell toSearch);


};

} /* namespace planner */

#endif /* INCLUDE_COMPLEXPLANNER_H_ */
