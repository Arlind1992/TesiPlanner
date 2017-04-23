/*
 * Baseline.h
 *
 *  Created on: Apr 22, 2017
 *      Author: arlind
 */

#ifndef BASELINE_H_
#define BASELINE_H_

#include "Planner.h"
#include "grid/Grid.h"
#include "grid/Cell.h"
#include "planners/AbstractPlanner.h"
#include <lemon/list_graph.h>
#include <lemon/dim2.h>
#include <planners/GridPlanner.h>
#include <fstream>

typedef lemon::ListDigraph DiGraph;
using namespace rrt_planning;
namespace planner {

class Baseline{
public:
	Baseline(rrt_planning::Grid* grid,int baseU,int baseR,const int buffer,std::ofstream* myfile):buffer(buffer),grid(grid),movingTime(0),length(graphAllNodes),allNodePoint(graphAllNodes),lengthBiggerRate(this->graphNodesBiggerRate){
		this->baseUnit=baseU;
		this->baseRate=baseR;

		this->myfile=myfile;


	}
	virtual ~Baseline();

	 bool makePlan(rrt_planning::Cell cgoal,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result);
	 void createGraphs();

private:

     //program attributes
     const int buffer;
     	    int baseUnit;
     	    int baseRate;
     	   int findPosition(std::vector<DiGraph::Node> nodes,DiGraph::Node n);
		/*
		 * Attributes
		 */
		Grid* grid;
	    //graphs that are going to be used to find the optimal path
		DiGraph graphAllNodes;
	    DiGraph graphNodesBiggerRate;
	    int movingTime;
	    double transmittionTime;

	    //maps for each node for all nodes graphs
	    DiGraph::ArcMap<int> length;
	    DiGraph::NodeMap<lemon::dim2::Point<int> > allNodePoint;
	    std::map<Cell,DiGraph::Node> cellAllNodes;


	    //maps for bigger rate Graph
	    DiGraph::ArcMap<int> lengthBiggerRate;
	    //TODO maybe delete
	    DiGraph::NodeMap<lemon::dim2::Point<int> > biggerRateNodePoint;



	    std::map<DiGraph::Node,DiGraph::Node> complexToNormal;
	    std::map<Cell,DiGraph::Node> cellBiggerRateNodes;
	    //Variables needed for the complex case graph

	    //map to get the expanded nodes from the node on the normal graph
	    std::map<DiGraph::Node,std::vector<DiGraph::Node>> nodeToVec;


	    /*
	     * helper functions
	     */
	     bool searchCell(std::vector<Cell> cells,Cell toSearch);

	     /*
	      * function to create the complex case graph with the expanded communication nodes and
	      * their connections
	      */
	     void createBiggerRateGraph();
	     /*
	      * functions to help creating the complex graph
	      */
	     void createBiggerRateNodes();
	     //create connection between arcs of the same normal graph nodes
	     void connectBiggerRateNodes();
	     void connectCellBiggerRate(Cell cell);
	     void connectBiggerRateCells(Cell start,Cell goal);
	     void createCellNodeBiggerRate();


	     /*
	      * functions to create the normal graph read it from a file if the file exists and asign
	      * it to the graph attribute
	      * or create it and then save it in a file
	      */
	     void createAllNodesGraph();
	     void createNodes();
	     void connectAllGraphNodes();
	     //function to create the cell node map
	     void createCellNode();

	     std::ofstream* myfile;
	     bool makePlanAllNodes(Cell start,Cell end,std::vector<Cell> &result);
	     void makePlanAllNodes(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances);

};

} /* namespace planner */

#endif /* BASELINE_H_ */
