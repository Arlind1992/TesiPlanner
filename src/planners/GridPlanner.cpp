/*
 * GridPlanner.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: arlind
 */

#include <lemon/dim2.h>
#include<lemon/dijkstra.h>
#include <planners/GridPlanner.h>
#include "grid/Grid.h"
using namespace lemon;
using namespace rrt_planning;
namespace planner {

bool GridPlanner::makePlan(Cell start,Cell goal,std::vector<Cell>& path,int buffer){


	GrGraph::Node firstNode=this->cellNode[start];
	GrGraph::Node endNode=this->cellNode[goal];
	lemon::Dijkstra<GrGraph,GrGraph::EdgeMap<int> > solver(this->graph,this->length);
	solver.run(firstNode,endNode);
	if(graph.id(solver.predNode(endNode))==-1){
			return false;
		}
	for (GrGraph::Node v=endNode;v != firstNode; v=solver.predNode(v)) {
		path.push_back(std::make_pair(this->nodePoint[v].x,this->nodePoint[v].y));
	}
	path.push_back(start);
	reverse(path.begin(),path.end());
	return true;

}


void GridPlanner::createGraph(){
	createNodes();
	createCellNode();
	connectNodes();
}

void GridPlanner::createNodes(){
	for(int i=0;i<grid->getMaxX();i++){
		for(int j=0;j<grid->getMaxY();j++){
			if(grid->isFree(std::make_pair(i,j))){
			GrGraph::Node addedNode = graph.addNode();
			lemon::dim2::Point<int> p(i,j);
			this->nodePoint[addedNode]=p;
			}
		}
	}
}
void GridPlanner::createCellNode(){
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			this->cellNode[std::make_pair(p.x,p.y)]=n;
		}
}

void GridPlanner::connectNodes(){
	std::vector<GrGraph::Node> checked;
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
		std::vector<Cell> neighbourCells=this->grid->getFourNeighbours(std::make_pair(this->nodePoint[n].x,this->nodePoint[n].y));
		checked.push_back(n);
		for(Cell c:neighbourCells){
			GrGraph::Node toConnect=this->cellNode[c];
			GrGraph::Edge addedEdge=graph.addEdge(n,toConnect);
			length[addedEdge]=1;
		}
	}
}
GridPlanner::~GridPlanner(){

}



} /* namespace planner */
