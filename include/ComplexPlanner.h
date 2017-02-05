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
#include <lemon/dim2.h>
typedef lemon::ListDigraph DiGraph;
using namespace rrt_planning;
namespace planner {

class ComplexPlanner{
public:
	ComplexPlanner(rrt_planning::Grid* grid):grid(grid),planner(grid),length(graph),buffer(0),nodePoint(graph),lengthComplex(this->complexCaseGraph),discretizationPar(1){
		this->numberOfNodes=(TMAX/this->discretizationPar)+1;
	}
	virtual ~ComplexPlanner();
	 bool makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cnit
		    		,std::vector<rrt_planning::Cell>& result);


private:


		/*
		 * Attributes
		 */
		Grid* grid;
	    //graph that is going to be used to find the optimal path
	    lemon::ListDigraph graph;
	    //maybe i need this complex case graph
	    DiGraph complexCaseGraph;
	    ThetaStarPlanner planner;
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
	    int buffer;
	    double discretizationPar;
	    //parameter to indicate the number of nodes in the complex graph for each node in the simple
	    //graph
	    int numberOfNodes;
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
