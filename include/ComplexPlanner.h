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
typedef lemon::ListDigraph DiGraph;
namespace planner {

class ComplexPlanner{
public:
	ComplexPlanner(rrt_planning::Grid* grid):grid(grid),planner(grid),length(graph),buffer(0),discretizationPar(1){

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
	    lemon::ListDigraph graph;
	    //maybe i need this complex case graph
	    DiGraph complexCaseGraph;
	    rrt_planning::ThetaStarPlanner planner;
	    DiGraph::ArcMap<double> length;
	    std::map<DiGraph::Arc,std::vector<rrt_planning::Cell> > arcPath;
	    std::map<DiGraph::Node,rrt_planning::Cell> nodeCell;
	    std::map<rrt_planning::Cell,DiGraph::Node> cellNode;

	    //map to keep track the node from the graph of the simple
	    //problem case from which the expanded nodes came from
	    std::map<std::vector<DiGraph::Node>,DiGraph::Node> vecToNode;
	    //map to get the expanded nodes from the node on the normal graph
	    std::map<std::vector<DiGraph::Node>,DiGraph::Node> nodeToVec;
	    //map to indicate if an arc is a moving arc or not
	    std::map<DiGraph::Arc,bool> isMoving;
	    //map to get from a couple of nodes the arc connecting them
	    std::map<std::pair<DiGraph::Node,DiGraph::Node>,DiGraph::Arc> nodesToArc;
	    int buffer;
	    int discretizationPar;
	    /*
	     * helper functions
	     */
	     bool searchCell(std::vector<rrt_planning::Cell> cells,rrt_planning::Cell toSearch);

	     /*
	      * function to create the complex case graph with the expanded communication nodes and
	      * their connections
	      */
	     void createComplexGraph();


};

} /* namespace planner */

#endif /* INCLUDE_COMPLEXPLANNER_H_ */
