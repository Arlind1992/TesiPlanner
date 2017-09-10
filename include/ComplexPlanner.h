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
#include "planners/AbstractPlanner.h"
#include <lemon/list_graph.h>
#include <lemon/dim2.h>
#include <planners/GridPlanner.h>
#include <fstream>

typedef lemon::ListDigraph DiGraph;
using namespace rrt_planning;
namespace planner {

class ComplexPlanner{
public:
	ComplexPlanner(rrt_planning::Grid* grid,int baseU,int baseR,GridPlanner* planner,const int buffer,std::ofstream* myfile):buffer(buffer),grid(grid),movingTime(0),length(graph),nodePoint(graph),lengthComplex(this->complexCaseGraph){
		this->baseUnit=baseU;
		this->baseRate=baseR;

		this->numberOfNodes=(buffer*(baseUnit/this->baseRate))+1;
		this->planner=planner;

		this->myfile=myfile;


	}
	virtual ~ComplexPlanner();
	 bool makePlan(rrt_planning::Cell cgoal,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result,std::map<Cell,int>& stateOfBuffer);
	 void createGraphs();
     bool makeSimplePlan(Cell cgoal,Cell cinit,std::vector<Cell>& result);


private:

     //program attributes
     const int buffer;
     	    int baseUnit;
     	    int baseRate;
     	    //parameter to indicate the number of nodes in the complex graph for each node in the simple
     	    //graph
     	    int numberOfNodes;
     	   int findPosition(std::vector<DiGraph::Node> nodes,DiGraph::Node n);
		/*
		 * Attributes
		 */
		Grid* grid;
	    //graphs that are going to be used to find the optimal path
		//graph created using the theta* planner
		DiGraph graph;
	    DiGraph complexCaseGraph;
	    int movingTime;
	    double transmittionTime;

	    //Planners
	    GridPlanner* planner;
	   // Grid* planner;
	    //maps for each node for different graphs
	    DiGraph::ArcMap<int> length;
	    DiGraph::NodeMap<lemon::dim2::Point<int> > nodePoint;
	    std::map<Cell,DiGraph::Node> cellNode;
	    DiGraph::ArcMap<int> lengthComplex;
	    std::map<DiGraph::Node,DiGraph::Node> complexToNormal;

	    //Variables needed for the complex case graph

	    //map to get the expanded nodes from the node on the normal graph
	    std::map<DiGraph::Node,std::vector<DiGraph::Node>> nodeToVec;


	    std::map<DiGraph::Node,DiGraph::Node> lastNodeToNormal;
	    std::map<Cell,DiGraph::Node> cellLastNode;

	    /*
	     * helper functions
	     */
	     bool searchCell(std::vector<Cell> cells,Cell toSearch);

	     /*
	      * function to create the complex case graph with the expanded communication nodes and
	      * their connections
	      */
	     void createComplexGraph();
	     /*
	      * functions to help creating the complex graph
	      */
	     void createComplexNodes();
	     //create connection between arcs of the same normal graph nodes
	     void createArcsSameNodes();
	     void connectDifferentNodes();
	     void connectFirstCellComplex(Cell cell);
	     void connectGoalCellComplex(Cell goal);
	     void connectComplexCells(Cell start,Cell goal);


	     /*
	      * functions to create the normal graph read it from a file if the file exists and asign
	      * it to the graph attribute
	      * or create it and then save it in a file
	      */
	     void createNormalGraph();
	     void createNodes();
	     void connectNormalNodes();
	     //function to create the cell node map
	     void createCellNode();
	     //function to create the node corresponding to the first cell and last cell
	     void connectCell(Cell cell);
	     //function to connect directly the first cell with the last cell if
	     //connection is possible and the cells are not communication cells
	     void connectCells(Cell cell1,Cell cell2);

	     char* casePl;
	     std::ofstream* myfile;


};

} /* namespace planner */

#endif /* INCLUDE_COMPLEXPLANNER_H_ */
