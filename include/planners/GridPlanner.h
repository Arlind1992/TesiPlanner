/*
 * GridPlanner.h
 *
 *  Created on: Feb 9, 2017
 *      Author: arlind
 */
#include "grid/Grid.h"
#include <lemon/dim2.h>
#include <lemon/list_graph.h>
#include "planners/AbstractPlanner.h"
#ifndef GRIDPLANNER_H_
#define GRIDPLANNER_H_
using namespace rrt_planning;
typedef lemon::ListGraph GrGraph;
namespace planner {

class GridPlanner : public AbstractPlanner
{
public:
	GridPlanner(Grid* grid):grid(grid),length(graph),nodePoint(graph){
		createGraph();
	}
    ~GridPlanner();
	virtual bool makePlan(Cell start,Cell goal,std::vector<Cell>& path,int buffer) override;

	void makePlan(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances);
private:

	Grid* grid;
	GrGraph graph;
	GrGraph::EdgeMap<int> length;
	GrGraph::NodeMap<lemon::dim2::Point<int> > nodePoint;
	//std::map<Cell,GrGraph::Node> cellNode;
	void createNodes();
	void connectNodes();
	void createCellNode();
	bool searchNode(std::vector<GrGraph::Node> list,GrGraph::Node tocheck);
	GrGraph::Node getNodeFromCell(Cell s);
	void createGraph();
};

} /* namespace planner */

#endif /* GRIDPLANNER_H_ */
