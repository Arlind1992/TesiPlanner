/*
 * Baseline.h
 *
 *  Created on: Apr 22, 2017
 *      Author: arlind
 */

#ifndef BASELINE_H_
#define BASELINE_H_

#include "grid/Grid.h"
#include "grid/Cell.h"
#include "planners/BaseLinePlanner.h"
#include "planners/GridPlanner.h"
#include <lemon/list_graph.h>
#include <lemon/dim2.h>
#include <fstream>

typedef lemon::ListDigraph BaGraph;
using namespace rrt_planning;
namespace planner {

class Baseline{
public:
	Baseline(rrt_planning::Grid* grid,int baseU,int baseR,const int buffer,std::ofstream* myfile,BaseLinePlanner *planner,GridPlanner *grPlan):buffer(buffer),grid(grid),blPlanner(planner)
,length(graph),nodePoint(graph),grPlanner(grPlan){
		this->baseUnit=baseU;
		this->baseRate=baseR;

		this->myfile=myfile;


	}
	virtual ~Baseline();

	 bool makePlan(rrt_planning::Cell cgoal,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result);
	 void createGraph();
private:

     //program attributes
     const int buffer;
     	    int baseUnit;
     	    int baseRate;
		/*
		 * Attributes
		 */
		Grid* grid;

	    BaGraph graph;

	    BaseLinePlanner * blPlanner;

	    GridPlanner *grPlanner;
	    //maps for bigger rate Graph
	    BaGraph::ArcMap<int> length;

	    BaGraph::NodeMap<lemon::dim2::Point<int> > nodePoint;

	    std::map<Cell,BaGraph::Node> cellNodes;



	     void createNodes();
	     //create connection between arcs of the same normal graph nodes
	     void connectNodes();
	     void createCellNode();

	     void connectNodesGrPlanner();

	     void connectStartCell(Cell cell);
	     void connectGoalCell(Cell cell);
	     void connectCells(Cell start,Cell goal);

	     std::ofstream* myfile;
	     double calculateTime(std::vector<Cell> commCells,std::vector<int> costToNext);
	     bool needToUpload(std::vector<int> buffStates);
	     void updateBufferStates(std::vector<int> *buffStates,int uploaded,int at);
	     int calculatePathCost(std::vector<int> path,int beg,int end);
};

} /* namespace planner */

#endif /* BASELINE_H_ */
