/*
 * ComplexPlanner.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: arlind
 */

#include "ComplexPlanner.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include<lemon/dijkstra.h>
//Assumtions : the buffer is starts always at empty
namespace planner {

ComplexPlanner::~ComplexPlanner() {
	// TODO Auto-generated destructor stub
}
#include <iostream>
#include <eigen3/Eigen/Dense>
#include<lemon/dijkstra.h>
using namespace std;
using namespace rrt_planning;
using namespace lemon;
using namespace planner;


bool planner::ComplexPlanner::makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cinit
		,vector<rrt_planning::Cell>& result){

	std::vector<rrt_planning::Cell> toCheck;
	toCheck.push_back(cinit);
	std::vector<rrt_planning::Cell> checked;
	DiGraph::Node node=graph.addNode();
	cellNode[cinit]=node;
	nodeCell[node]=cinit;
	int i=0;
	while(!toCheck.empty()){
		i++;
		std::cout<<i<<std::endl;
		rrt_planning::Cell cell=toCheck.front();
		toCheck.erase(toCheck.begin());
		checked.push_back(cell);
		std::vector<rrt_planning::Cell> comm_cells=grid->commCells(cell,Tmax,cgoal);
		std::vector<rrt_planning::Cell>::iterator it;
		for(it=comm_cells.begin();it<comm_cells.end();it++){
			if(!searchCell(checked,*it)){
				std::vector<rrt_planning::Cell> pathTo;
				planner.makePlan(cell,*it,pathTo);
				double distance=grid->pathCost(pathTo);
				if(distance<=Tmax){
					try{
						node=cellNode.at(*it);
						}catch(const std::out_of_range& oor){
						node=graph.addNode();
						cellNode[*it]=node;
						nodeCell[node]=*it;
					}

						 if(!searchCell(toCheck,*it)){

										    	toCheck.push_back(*it);
						 }
						 DiGraph::Node toconnect=cellNode[cell];
					DiGraph::Arc arc=graph.addArc(toconnect,node);
					this->nodesToArc[std::make_pair(toconnect,node)]=arc;
					DiGraph::Arc inverseArc=graph.addArc(node,toconnect);
					length[inverseArc]=distance;
					std::vector<rrt_planning::Cell> reversedPath=pathTo;
					std::reverse(reversedPath.begin(),reversedPath.end());
					arcPath[inverseArc]=pathTo;
					length[arc]=distance;
					arcPath[arc]=pathTo;
				}
			}
		}

	}
	try{
	cellNode.at(cgoal);
	}catch(const std::out_of_range& oor){
		return false;
	}
	/*
	 * test graph TODO delete
	 * after testing check the length of the path or dijkstra
	 *
	for( ListGraph::EdgeIt n(graph); n != INVALID; ++n ){
	    std::vector<Cell> toPrint=edgePath[n];
	    Planner::stampVector(toPrint);
	    double cost=length[n];
	    std::cout<<cost<<std::endl;
		std::cout<<"end"<<std::endl;
	}
	int i=1;
	std::cout<<"begin nodes"<<std::endl;
	for( ListGraph::NodeIt n(graph); n != INVALID; ++n ){

		    std::cout<<i<<"-"<<"("<<nodeCell[n].first<<","<<nodeCell[n].second<<")"<<std::endl;
	i++;
	}*/
	std::cout<<"end nodes"<<std::endl;
	this->createComplexGraph();
	lemon::Dijkstra<DiGraph,DiGraph::ArcMap<double>> solver(this->complexCaseGraph,length);
	solver.run(cellNode[cinit],cellNode[cgoal]);
    for (DiGraph::Node v=cellNode[cgoal];v != cellNode[cinit]; v=solver.predNode(v)) {
      DiGraph::Node prevNode=solver.predNode(v);
      DiGraph::Arc edge= lemon::findArc(graph,prevNode,v,lemon::INVALID);
      std::vector<rrt_planning::Cell> vec=arcPath[edge];
      reverse(vec.begin(),vec.end());
      result.insert(result.end(),vec.begin(),vec.end());
    }
    reverse(result.begin(),result.end());
    result.erase(unique(result.begin(),result.end()),result.end());

	return true;
}
bool planner::ComplexPlanner::searchCell(std::vector<rrt_planning::Cell> cells,rrt_planning::Cell toSearch){
	for(rrt_planning::Cell cell :cells){
		if(cell.first==toSearch.first&&cell.second==toSearch.second){
			return true;
		}
	}
	return false;
}

void planner::ComplexPlanner::createComplexGraph(){
	//TODO maybe add rrumbullakimin per siper
	int numOfNodes=(TMAX/this->discretizationPar)+1;
	//TODO add nodes for beginning and end cells
	/*
	 * for loop that goes through all the nodes of the graph we want to transform
	 * checks if the cell associated with the node is a communication node
	 * then transforms it
	 */
	for(DiGraph::NodeIt n(graph); n != INVALID; ++n ){
		int speed;
		std::vector<DiGraph::Node> vecNodes;
		if(this->grid->isComm(this->nodeCell[n],&speed)){
			for(int i=0;i<numOfNodes;i++){
				DiGraph::Node addNode=this->complexCaseGraph.addNode();
				vecNodes.push_back(addNode);
				this->vecToNode[vecNodes]=n;
				this->nodeToVec[n]=vecNodes;
			}
		}
	}
	/*
	 * for loop that creates the connections between the nodes in the
	 * complex case
	 */
	for(DiGraph::NodeIt n(graph); n != INVALID; ++n ){
			int speed;
			if(this->grid->isComm(this->nodeCell[n],&speed)){
			std::vector<DiGraph::Node> vecStartNodes=nodeToVec[n];
			//for loops to create connections between comm nodes associated to same cell
			    for(int i=1;i<speed;i++){
					DiGraph::Arc arc=this->complexCaseGraph.addArc(vecStartNodes.at(i),vecStartNodes.at(0));
					this->length[arc]=this->discretizationPar;
					this->isMoving[arc]=false;
				}
			    for(int i=0;i<=numOfNodes-speed;i++){
			    	DiGraph::Arc toadd=this->complexCaseGraph.addArc(vecStartNodes.at(i+speed),vecStartNodes.at(i));
			    	this->length[toadd]=this->discretizationPar;
			    	this->isMoving[toadd]=false;
			    }
			    //for loop to connect with the other nodes
				for(DiGraph::OutArcIt out(graph,n);out!=INVALID;++out){
					//next node in the normal graph connected
					DiGraph::Node nod=graph.target(out);
					//normal graph arc from which to receive length and paths
					DiGraph::Arc normalGraphArc=this->nodesToArc[std::make_pair(n,nod)];
					std::vector<rrt_planning::Cell> path=this->arcPath[normalGraphArc];
					int distance=this->length[normalGraphArc];
					std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
					for(int j=0;j<numOfNodes-distance;j++){
						DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecStartNodes.at(j),vecEndNodes.at(j+distance));
						this->length[addedArc]=distance;
						this->arcPath[addedArc]=path;
						this->isMoving[addedArc]=true;
					}
				}
			}
		}
}



} /* namespace planner */
