/*
 * Planner.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: arlind
 */


#include "Planner.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include<lemon/dijkstra.h>
using namespace std;
using namespace rrt_planning;
using namespace lemon;
using namespace planner;


bool planner::Planner::makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cinit
		,vector<rrt_planning::Cell>& result){

	std::vector<rrt_planning::Cell> toCheck;
	toCheck.push_back(cinit);
	std::set<rrt_planning::Cell> checked;
	Graph::Node node=graph.addNode();
	//TODO probably don't need it so delete
	cellNode[cinit]=node;
	//TODO maybe delete
	nodeCell[node]=cinit;
	while(!toCheck.empty()){
		rrt_planning::Cell cell=toCheck.front();
		toCheck.erase(toCheck.begin());
		checked.insert(cell);
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
						//TODO maybe delete
						nodeCell[node]=*it;
					}

					toCheck.push_back(*it);
					Graph::Node toconnect=cellNode[cell];
					Graph::Edge edge=graph.addEdge(node,toconnect);
					length[edge]=distance;
					edgePath[edge]=pathTo;
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
	 */
	for( ListGraph::EdgeIt n(graph); n != INVALID; ++n ){
	    std::vector<Cell> toPrint=edgePath[n];
	    this->stampVector(toPrint);
	    double cost=length[n];
	    std::cout<<cost<<std::endl;
		std::cout<<"end"<<std::endl;
	}
	int i=1;
	std::cout<<"begin nodes"<<std::endl;
	for( ListGraph::NodeIt n(graph); n != INVALID; ++n ){

		    std::cout<<i<<"-"<<"("<<nodeCell[n].first<<","<<nodeCell[n].second<<")"<<std::endl;
	i++;
	}
	std::cout<<"end nodes"<<std::endl;


	lemon::Dijkstra<Graph,Graph::EdgeMap<double>> solver(graph,length);
	solver.run(cellNode[cinit],cellNode[cgoal]);
    for (Graph::Node v=cellNode[cgoal];v != cellNode[cinit]; v=solver.predNode(v)) {
      Graph::Node prevNode=solver.predNode(v);
      Graph::Edge edge= lemon::findEdge(graph,prevNode,v,lemon::INVALID);
      std::vector<rrt_planning::Cell> vec=edgePath[edge];
      reverse(vec.begin(),vec.end());
      result.insert(result.end(),vec.begin(),vec.end());
    }
    reverse(result.begin(),result.end());
    result.erase(unique(result.begin(),result.end()),result.end());

	return true;
}
bool planner::Planner::searchCell(std::set<rrt_planning::Cell> cells,rrt_planning::Cell toSearch){
	for(rrt_planning::Cell cell :cells){
		if(cell.first==toSearch.first&&cell.second==toSearch.second){
			return true;
		}
	}
	return false;
}
 void planner::Planner::stampVector(std::vector<Cell> cells){
	std::cout<<std::endl;
	 for(Cell c:cells){
		std::cout<<"("<<c.first<<","<<c.second<<")"<<std::endl;
	}
}


