

#ifndef PLANNING_H_
#define PLANNING_H_

#include "grid/Grid.h"
#include "grid/Cell.h"
#include "rrt_planning/ThetaStarPlanner.h"
#include <lemon/list_graph.h>
typedef lemon::ListGraph Graph;
typedef Graph::EdgeMap<int> LengthMap;
#define TMAX 10
namespace planner{
class Planner{
public :
	static void stampVector(std::vector<rrt_planning::Cell> cells);
	Planner(rrt_planning::Grid* grid):grid(grid),planner(grid),length(graph){
	}
	~Planner(){
	}
	 bool makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cnit
	    		,std::vector<rrt_planning::Cell>& result);

private :


	/*
	 * Attributes
	 */
	rrt_planning::Grid* grid;
    //graph that is going to be used to find the optimal path
    lemon::ListGraph graph;
    rrt_planning::ThetaStarPlanner planner;
    Graph::EdgeMap<double> length;
    std::map<Graph::Edge,std::vector<rrt_planning::Cell> > edgePath;
    std::map<Graph::Node,rrt_planning::Cell> nodeCell;
    //TODO probably don't need it so delete
    std::map<rrt_planning::Cell,Graph::Node> cellNode;

    /*
     * helper functions
     */
     bool searchCell(std::vector<rrt_planning::Cell> cells,rrt_planning::Cell toSearch);





};

}
#endif
