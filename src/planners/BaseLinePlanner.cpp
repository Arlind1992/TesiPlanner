/*
 * BaseLinePlannerPlanner.cpp
 *
 *  Created on: Apr 25, 2017
 *      Author: arlind
 */

#include "planners/BaseLinePlanner.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include<lemon/dijkstra.h>
#include <lemon/dim2.h>
using namespace std;
using namespace rrt_planning;
using namespace lemon;

planner::BaseLinePlanner::~BaseLinePlanner() {
	// TODO Auto-generated destructor stub
}

//WORKS
bool planner::BaseLinePlanner::makePlan(Cell start,Cell end,std::vector<Cell> &result,int *cost,int* bufferCost){
	LiGraph::Node firstNode=this->getNodeFromCell(start);
		LiGraph::Node endNode=this->getNodeFromCell(end);
		int counter=0;
		lemon::Dijkstra<LiGraph,LiGraph::EdgeMap<int> > solver(this->graphAllNodes,this->length);
		solver.run(firstNode,endNode);
		if(graphAllNodes.id(solver.predNode(endNode))==-1){
				return false;
			}
		for (LiGraph::Node v=endNode;v != firstNode; v=solver.predNode(v)) {
			counter++;
			lemon::dim2::Point<int> p=nodePoint[v];
			result.push_back(std::make_pair(p.x,p.y));
		}
		*cost=counter;
		*bufferCost=solver.dist(endNode);
		result.push_back(start);
		reverse(result.begin(),result.end());
		return true;

}

void planner::BaseLinePlanner::makePlanAllNodes(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances){
	LiGraph::Node firstNode=getNodeFromCell(start);
	lemon::Dijkstra<LiGraph,LiGraph::EdgeMap<int> > solver(this->graphAllNodes,this->length);
		solver.run(firstNode);
	for(LiGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
		//std::cout<<"after"<<std::endl;
		lemon::dim2::Point<int> p=nodePoint[n];
		if(!grid->isComm(std::make_pair(p.x,p.y))||grid->getSpeed(std::make_pair(p.x,p.y))<=baseUnit)
			continue;
		if(solver.dist(n)<=buffer*baseUnit){
			cells.push_back(std::make_pair(p.x,p.y));
			int count=0;
			for (LiGraph::Node v=n;v != firstNode; v=solver.predNode(v)) {
				count++;
			}
			distances.push_back(count);
		}
	}
}

void planner::BaseLinePlanner::makePlanAllNodesDistanceAsCost(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances){
	LiGraph::Node firstNode=getNodeFromCell(start);
	lemon::Dijkstra<LiGraph,LiGraph::EdgeMap<int> > solver(this->graphAllNodes,this->length);
		solver.run(firstNode);
	std::vector<Cell> gridcells;
	std::vector<int> griddistances;
	gridPlanner->makePlan(start,gridcells,buffer,griddistances);
	for(LiGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
		//std::cout<<"after"<<std::endl;
		lemon::dim2::Point<int> p=nodePoint[n];
		if(!grid->isComm(std::make_pair(p.x,p.y))||grid->getSpeed(std::make_pair(p.x,p.y))<=baseUnit)
			continue;
		if(solver.dist(n)<=buffer*baseUnit){
			cells.push_back(std::make_pair(p.x,p.y));
			for(int j=0;j<gridcells.size();j++){
				if(p.x==gridcells.at(j).first&&p.y==gridcells.at(j).second)
				distances.push_back(griddistances.at(j));
			}

		}
	}
}




/*
 * Done
 */
void planner::BaseLinePlanner::createGraph(){
	createNodes();
	//createCellNode();
	this->connectAllGraphNodes();

}
void planner::BaseLinePlanner::createNodes(){
	for(int i=0;i<grid->getMaxX();i++){
		for(int j=0;j<grid->getMaxY();j++){
			if(this->grid->isFree(std::make_pair(i,j))){
				LiGraph::Node addedNode=this->graphAllNodes.addNode();
				lemon::dim2::Point<int> p(i,j);
				this->nodePoint[addedNode]=p;
			}
		}
	}
}
/*
void planner::BaseLinePlanner::createCellNode(){
	for(LiGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		this->cellNodes[std::make_pair(p.x,p.y)]=n;
	}
}*/

void planner::BaseLinePlanner::connectAllGraphNodes(){
		for(LiGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			std::vector<Cell> neighbourCells=this->grid->getFourNeighbours(std::make_pair(p.x,p.y));
			for(Cell c:neighbourCells){
				LiGraph::Node toConnect=getNodeFromCell(c);
				LiGraph::Edge addedEdge=graphAllNodes.addEdge(n,toConnect);
				if(!grid->isComm(std::make_pair(p.x,p.y))||!grid->isComm(c)){
					length[addedEdge]=baseUnit;
				}else{
					if(grid->getSpeed(c)<=grid->getSpeed(std::make_pair(p.x,p.y))){
						if(grid->getSpeed(c)>=baseUnit){
							length[addedEdge]=0;
						}else{
							length[addedEdge]=baseUnit-grid->getSpeed(c);
						}
					}else{
						if(grid->getSpeed(std::make_pair(p.x,p.y))>=baseUnit){
							length[addedEdge]=0;
						}else{
							length[addedEdge]=baseUnit-grid->getSpeed(std::make_pair(p.x,p.y));
						}
					}
				}
			}
		}
}

void planner::BaseLinePlanner::connectAllGraphNodesDistanceAsCost(){
		for(LiGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			std::vector<Cell> neighbourCells=this->grid->getFourNeighbours(std::make_pair(p.x,p.y));
			for(Cell c:neighbourCells){
				LiGraph::Node toConnect=getNodeFromCell(c);
				LiGraph::Edge addedEdge=graphAllNodes.addEdge(n,toConnect);
				if(!grid->isComm(std::make_pair(p.x,p.y))||!grid->isComm(c)){
					length[addedEdge]=baseUnit;
				}else{
					if(grid->getSpeed(c)<=grid->getSpeed(std::make_pair(p.x,p.y))){
						if(grid->getSpeed(c)>=baseUnit){
							length[addedEdge]=0;
						}else{
							length[addedEdge]=baseUnit-grid->getSpeed(c);
						}
					}else{
						if(grid->getSpeed(std::make_pair(p.x,p.y))>=baseUnit){
							length[addedEdge]=0;
						}else{
							length[addedEdge]=baseUnit-grid->getSpeed(std::make_pair(p.x,p.y));
						}
					}
				}
			}
		}
}

LiGraph::Node planner::BaseLinePlanner::getNodeFromCell(Cell s){
	for(LiGraph::NodeIt n(this->graphAllNodes);n!=INVALID;++n){
	if(nodePoint[n].x==s.first&&nodePoint[n].y==s.second){
		return n;
	}
	}
}




