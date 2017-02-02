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

//TODO make map 50 per 50
bool planner::Planner::makePlan(rrt_planning::Cell cgoal,int Tmax,rrt_planning::Cell cinit
		,vector<rrt_planning::Cell>& result){

	std::vector<rrt_planning::Cell> toCheck;
	toCheck.push_back(cinit);
	std::vector<rrt_planning::Cell> checked;
	Graph::Node node=graph.addNode();
	//TODO probably don't need it so delete
	cellNode[cinit]=node;

	std::vector<rrt_planning::Cell>::iterator it;

	//TODO maybe delete
	nodeCell[node]=cinit;
	int numi=0;
	int numj=0;
	while(!toCheck.empty()){
		numi++;
		rrt_planning::Cell cell=toCheck.front();
		toCheck.erase(toCheck.begin());
		checked.push_back(cell);
		std::vector<rrt_planning::Cell> comm_cells=grid->commCells(cell,Tmax,cgoal);
		for(it=comm_cells.begin();it<comm_cells.end();it++){
			if(!searchCell(checked,*it)){
				numj++;
				std::vector<rrt_planning::Cell> pathTo;
			//TODO stampa tempi di theta*
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
				    if(!searchCell(toCheck,*it)){

				    	toCheck.push_back(*it);
					}


					Graph::Node toconnect=cellNode[cell];
					Graph::Edge edge=graph.addEdge(node,toconnect);
					length[edge]=distance;
					edgePath[edge]=pathTo;
				}
			}
		}

		comm_cells.clear();

	}
	std::cout<<"inside "<<numj<<std::endl;
	try{
	cellNode.at(cgoal);
	}catch(const std::out_of_range& oor){
		return false;
	}
	//count edges
	int k=0;
	for(Graph::EdgeIt it(graph);it!=INVALID;++it){
		k++;
	}
	std::cout<<"number fo edges"<<k<<std::endl;
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

    //TODO delete after test
	/*for(Graph::NodeIt n(graph); n != INVALID; ++n ){
				std::cout<<graph.id(n)<<std::endl;
				std::cout<<"representin:"<<"("<<this->nodeCell[n].first<<","<<this->nodeCell[n].second<<")"<<std::endl;
			}
   int z=0;
			for(Graph::ArcIt b(graph);b!=INVALID;++b){
				std::cout<<"Arc ="<<std::endl;
				std::cout<<"("<<nodeCell[(graph.source(b))].first<<","<<nodeCell[(graph.source(b))].second<<")"<<"-"<<"("<<nodeCell[(graph.target(b))].first<<","<<nodeCell[(graph.target(b))].second<<")"<<std::endl;

				std::cout<<"distance:"<<this->length[b]<<std::endl;
					std::cout<<"path:"<<std::endl;
					Planner::stampVector(this->edgePath[b]);
					std::cout<<graph.id(b)<<"-id"<<endl;
					std::cout<<"end Arc"<<std::endl;

					z++;

			}
			std::cout<<"number of arcs="<<z<<std::endl;
*/
	return true;
}
bool planner::Planner::searchCell(std::vector<rrt_planning::Cell> cells,rrt_planning::Cell toSearch){
	for(rrt_planning::Cell cell :cells){
		if(cell.first==toSearch.first&&cell.second==toSearch.second){
			return true;
		}
	}
	return false;
}
 void planner::Planner::stampVector(std::vector<Cell> cells){
	 if(cells.empty()){
		 std::cout<<"empty"<<std::endl;
	 }
	std::cout<<std::endl;
	 for(Cell c:cells){
		std::cout<<"("<<c.first<<","<<c.second<<")"<<std::endl;
	}
}


