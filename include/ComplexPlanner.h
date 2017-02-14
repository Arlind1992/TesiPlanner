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
typedef lemon::ListDigraph DiGraph;
using namespace rrt_planning;
namespace planner {

class ComplexPlanner{
public:
	ComplexPlanner(rrt_planning::Grid* grid,double disPar,AbstractPlanner* planner,char* filePath):grid(grid),virtualTime(0),length(graph),nodePoint(graph),lengthComplex(this->complexCaseGraph){
		this->discretizationPar=disPar;
		this->numberOfNodes=calculateNum(buffer/this->discretizationPar)+1;
		this->planner=planner;
		this->filePath=filePath;
	}
	virtual ~ComplexPlanner();
	 bool makePlan(rrt_planning::Cell cgoal,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result);
	 void createGraphs();
     bool makeSimplePlan(Cell cgoal,Cell cinit,std::vector<Cell>& result);

     //function to get the virtual time to be used after every make plan
     int getVirtualTime();
private:


		/*
		 * Attributes
		 */
		Grid* grid;
	    //graphs that are going to be used to find the optimal path
		//graph created using the theta* planner
		lemon::ListDigraph graph;
	    DiGraph complexCaseGraph;
	    double virtualTime;

	    //Planners
	   // ThetaStarPlanner planner;
	    AbstractPlanner* planner;
	    //maps for each node for different graphs
	    DiGraph::ArcMap<double> length;
	    DiGraph::NodeMap<lemon::dim2::Point<int> > nodePoint;
	    std::map<Cell,DiGraph::Node> cellNode;
	    DiGraph::ArcMap<double> lengthComplex;
	    std::map<DiGraph::Node,DiGraph::Node> complexToNormal;
	    //Variables needed for the complex case graph

	    //map to get the expanded nodes from the node on the normal graph
	    std::map<DiGraph::Node,std::vector<DiGraph::Node>> nodeToVec;
	    //map to indicate if an arc is a moving arc or not
	    std::map<DiGraph::Arc,bool> isMoving;
	    const int buffer=30;
	    double discretizationPar;
	    //parameter to indicate the number of nodes in the complex graph for each node in the simple
	    //graph
	    int numberOfNodes;

	    const char* filePath;
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
	     //function to calculate distance based on the discretization parameter
	     double calculateNum(double distance);

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






};

} /* namespace planner */

#endif /* INCLUDE_COMPLEXPLANNER_H_ */
