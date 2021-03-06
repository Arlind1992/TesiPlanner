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
void GridPlanner::makePlan(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances){
	GrGraph::Node firstNode=this->getNodeFromCell(start);
	lemon::Dijkstra<GrGraph,GrGraph::EdgeMap<int> > solver(this->graph,this->length);
		solver.run(firstNode);
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
		if(!grid->isComm(std::make_pair(nodePoint[n].x,nodePoint[n].y)))
			continue;
		if(solver.dist(n)<=buffer){
			cells.push_back(std::make_pair(nodePoint[n].x,nodePoint[n].y));
			distances.push_back(solver.dist(n));
		}
	}


}


bool GridPlanner::makePlan(Cell start,Cell goal,std::vector<Cell>& path,int buffer){


	GrGraph::Node firstNode=this->getNodeFromCell(start);
	GrGraph::Node endNode=this->getNodeFromCell(goal);
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
	//createCellNode();
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
/*
void GridPlanner::createCellNode(){
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			this->cellNode[std::make_pair(p.x,p.y)]=n;
		}
}*/

void GridPlanner::connectNodes(){
	std::vector<GrGraph::Node> checked;
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
		std::vector<Cell> neighbourCells=this->grid->getFourNeighbours(std::make_pair(this->nodePoint[n].x,this->nodePoint[n].y));
		checked.push_back(n);
		for(Cell c:neighbourCells){
			GrGraph::Node toConnect=getNodeFromCell(c);
			GrGraph::Edge addedEdge=graph.addEdge(n,toConnect);
			length[addedEdge]=1;
		}
	}
}
GridPlanner::~GridPlanner(){

}
//TODO if fixed memory problem delete
GrGraph::Node GridPlanner::getNodeFromCell(Cell s){
	for(GrGraph::NodeIt n(graph);n!=INVALID;++n){
	if(nodePoint[n].x==s.first&&nodePoint[n].y==s.second){
		return n;
	}
	}
}


} /* namespace planner */
