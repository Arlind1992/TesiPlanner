/*
 * BaseLinePlanner.h
 *
 *  Created on: Apr 25, 2017
 *      Author: arlind
 */

#ifndef PLANNERS_BASELINEPLANNER_H_
#define PLANNERS_BASELINEPLANNER_H_

#include "grid/Grid.h"
#include "grid/Cell.h"
#include "planners/BaseLinePlanner.h"
#include "planners/GridPlanner.h"
#include <lemon/list_graph.h>
#include <lemon/dim2.h>


typedef lemon::ListGraph LiGraph;
using namespace std;
using namespace rrt_planning;
using namespace lemon;
namespace planner {
class BaseLinePlanner {
public:
	BaseLinePlanner(rrt_planning::Grid* grid,int base,GridPlanner *grPlan):baseUnit(base),grid(grid),length(graphAllNodes),nodePoint(graphAllNodes),gridPlanner(grPlan){

	}
	virtual ~BaseLinePlanner();
	bool makePlan(Cell start,Cell end,std::vector<Cell> &result,int *cost,int *bufferCost);
		void makePlanAllNodes(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances);
		void makePlanAllNodesDistanceAsCost(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances);
		 void createGraph();
private:


	int baseUnit;

			Grid* grid;
			//graphs that are going to be used to find the optimal path
						LiGraph graphAllNodes;
			//maps for each node for all nodes graphs
	LiGraph::EdgeMap<double> length;
	LiGraph::NodeMap<lemon::dim2::Point<int> > nodePoint;
	//std::map<Cell,LiGraph::Node> cellNodes;
	GridPlanner *gridPlanner;


	/*
		      * functions to create the normal graph read it from a file if the file exists and asign
		      * it to the graph attribute
		      * or create it and then save it in a file
		      */
	void connectAllGraphNodesDistanceAsCost();

	void createNodes();
	void connectAllGraphNodes();
	//function to create the cell node map
	//void createCellNode();
	LiGraph::Node getNodeFromCell(Cell s);

};
}

#endif /* PLANNERS_BASELINEPLANNER_H_ */
