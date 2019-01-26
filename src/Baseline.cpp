/*
 * Baseline.cpp
 *
 *  Created on: Apr 22, 2017
 *      Author: arlind
 */

#include "Baseline.h"


/*
 * Baseline.cpp
 *
 *  Created on: Jan 21, 2017
 *      Author: arlind
 */

#include "Baseline.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include<lemon/dijkstra.h>
#include <lemon/dim2.h>
#include <typeinfo>
using namespace std;
using namespace rrt_planning;
using namespace lemon;
namespace planner {

Baseline::~Baseline() {

}
bool planner::Baseline::makePlan(Cell cgoal,Cell cinit
		,vector<Cell>& result ){
		int start_s=clock();
		if(!grid->isFree(cgoal)||!grid->isFree(cinit)){
				return false;
		}
		int movingTime=0;
		double	transmittionTime=0;
		bool isCommInit=grid->isComm(cinit)&&grid->getSpeed(cinit)>baseUnit;
		bool isCommGoal=grid->isComm(cgoal)&&grid->getSpeed(cgoal)>baseUnit;
		if(!isCommInit)
			this->connectStartCell(cinit);
		if(!isCommGoal)
			this->connectGoalCell(cgoal);
		if(!isCommInit&&!isCommGoal)
			this->connectCells(cgoal,cinit);
		lemon::Dijkstra<BaGraph,BaGraph::ArcMap<int>> solver(graph,length);
		BaGraph::Node first=cellNodes[cinit];
		BaGraph::Node end=cellNodes[cgoal];
		std::vector<Cell> toCalculateT;
		std::vector<int> costForT;
		toCalculateT.push_back(cgoal);
		costForT.push_back(0);
		solver.run(first,end);
		if(graph.id(solver.predNode(end))==-1){
			return false;
		}
		std::vector<Cell> vec;
		for (BaGraph::Node v=end;v != first; v=solver.predNode(v)) {
			 BaGraph::Node prevNode=solver.predNode(v);
			 Cell endCell;
			 lemon::dim2::Point<int> p=nodePoint[v];
			 endCell.first=p.x;
			 endCell.second=p.y;
			 Cell startCell;
			 lemon::dim2::Point<int> p2=nodePoint[prevNode];
			 startCell.first=p2.x;
			 startCell.second=p2.y;
			 int cost;
			 int buffCost;
			 blPlanner->makePlan(startCell,endCell,vec,&cost,&buffCost);
			 //vec=blPlanner->paths[std::make_pair(startCell,endCell)];
			 //vec.clear();
			 //grPlanner->makePlan(startCell,endCell,vec,buffCost);
			 for(int veci=0;veci<vec.size()-1;veci++){
				 if(grid->getSpeed(vec.at(veci))>baseUnit&grid->getSpeed(vec.at(veci+1))>baseUnit){
					 toCalculateT.push_back(vec.at(veci));
					 costForT.push_back(1);
				 }else{
					 if(grid->getSpeed(vec.at(veci))>baseUnit&grid->getSpeed(vec.at(veci+1))<=baseUnit){
						 toCalculateT.push_back(vec.at(veci));
						 costForT.push_back(buffCost);
					 }
				 }

			 }

			 reverse(vec.begin(),vec.end());
			 result.insert(result.end(),vec.begin(),vec.end());
			 vec.clear();
	    }
		reverse(toCalculateT.begin(),toCalculateT.end());
		reverse(costForT.begin(),costForT.end());
		transmittionTime=this->calculateTime(toCalculateT,costForT);
	    reverse(result.begin(),result.end());
	    result.erase(unique(result.begin(),result.end()),result.end());
	    int stop_s=clock();
	    movingTime=solver.dist(cellNodes[cgoal]);
	    if(!isCommInit){
	    	this->graph.erase(first);
	    	cellNodes.erase(cinit);
	    }
	    if(!isCommGoal){
		   	this->graph.erase(end);
		   	cellNodes.erase(cgoal);
	    }
	   (*myfile)<<"Baseline computational cost: "<<((stop_s-start_s)/double(CLOCKS_PER_SEC)*1000) <<"\n";
		(*myfile)<<"Baseline transmition cost : "<<transmittionTime<<"\n";
		(*myfile)<<"Baseline path cost : "<<movingTime<<"\n";
		return true;
}

void planner::Baseline::createGraph(){

	int s1=clock();
	this->createNodes();
	this->createCellNode();
	this->connectNodes();
	//connectNodesGrPlanner();
	int s2=clock();
	(*myfile)<<"Time to create Baseline graph (nodes with bigger Rate): "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<"\n";

}
//
void planner::Baseline::createCellNode(){
	for(BaGraph::NodeIt n(graph);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			this->cellNodes[std::make_pair(p.x,p.y)]=n;
		}
}


void planner::Baseline::connectStartCell(Cell cell){
	BaGraph::Node addedNode=graph.addNode();
	lemon::dim2::Point<int> p(cell.first,cell.second);
	this->nodePoint[addedNode]=p;
	this->cellNodes[cell]=addedNode;
	std::vector<Cell> cells;
	std::vector<int> distances;
	blPlanner->makePlanAllNodes(cell,cells,buffer,distances);
	for(int i=0;i<cells.size();i++){
		BaGraph::Node toConnect=this->cellNodes[cells.at(i)];
		BaGraph::Arc conn=graph.addArc(addedNode,toConnect);
		length[conn]=distances.at(i);
	}
	cells.clear();
	distances.clear();
}
void planner::Baseline::connectGoalCell(Cell cell){
	BaGraph::Node addedNode=graph.addNode();
	lemon::dim2::Point<int> p(cell.first,cell.second);
	this->nodePoint[addedNode]=p;
	this->cellNodes[cell]=addedNode;
	std::vector<Cell> cells;
	std::vector<int> distances;
	blPlanner->makePlanAllNodes(cell,cells,buffer,distances);
	for(int i=0;i<cells.size();i++){
		BaGraph::Node toConnect=this->cellNodes[cells.at(i)];
		BaGraph::Arc inv=graph.addArc(toConnect,addedNode);
		length[inv]=distances.at(i);
	}
	cells.clear();
	distances.clear();
}
//Checked
void planner::Baseline::createNodes(){
		for(int i=0;i<grid->getMaxX();i++){
			for(int j=0;j<grid->getMaxY();j++){
				if(this->grid->isComm(std::make_pair(i,j)) && this->grid->isFree(std::make_pair(i,j))&&this->grid->getSpeed(std::make_pair(i,j))>baseUnit){
					BaGraph::Node addedNode=this->graph.addNode();
					lemon::dim2::Point<int> p(i,j);
					this->nodePoint[addedNode]=p;
				}
			}
		}

}

//
void planner::Baseline::connectNodes(){
	for(BaGraph::NodeIt n(graph);n!=INVALID;++n){
			lemon::dim2::Point<int> p=nodePoint[n];
			std::vector<Cell> cells;
			std::vector<int> distances;
			blPlanner->makePlanAllNodes(std::make_pair(p.x,p.y),cells,buffer,distances);
			for(int i=0;i<cells.size();i++){
				BaGraph::Node toConnect=this->cellNodes[cells.at(i)];
				//std::cout<<"node distance ("<<p.x<<","<<p.y<<")"<<"->"<<"("<<cells.at(i).first<<","<<cells.at(i).second<<")="<<distances.at(i)<<std::endl;
				BaGraph::Arc conn=graph.addArc(n,toConnect);
				length[conn]=distances.at(i);

			}
			cells.clear();
			distances.clear();

		}


}


void planner::Baseline::connectNodesGrPlanner(){
	for(BaGraph::NodeIt n(graph);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		std::vector<Cell> cells;
		std::vector<int> distances;
		blPlanner->makePlanAllNodesDistanceAsCost(std::make_pair(p.x,p.y),cells,buffer,distances);
		for(int i=0;i<cells.size();i++){
			BaGraph::Node toConnect=this->cellNodes[cells.at(i)];
			BaGraph::Arc conn=graph.addArc(n,toConnect);
			length[conn]=distances.at(i);

		}
		cells.clear();
		distances.clear();

	}
}



void planner::Baseline::connectCells(Cell start,Cell goal){
		int cost;
		int buffCost;
		std::vector<Cell> sol;
		blPlanner->makePlan(start,goal,sol,&cost,&buffCost);
		if(buffCost<=buffer*baseUnit){
			BaGraph::Arc arc=graph.addArc(cellNodes[start],cellNodes[goal]);
			BaGraph::Arc invArc=graph.addArc(cellNodes[goal],cellNodes[start]);
			this->length[invArc]=cost;
			this->length[arc]=cost;
		}
}

double Baseline::calculateTime(std::vector<Cell> commCells,std::vector<int> cost){
         std::vector<int> modifiedCosts;
         for(int i=0;i<commCells.size()-1;i++){
        	 if(cost.at(i)==0&&grid->areFourConnected(commCells.at(i),commCells.at(i+1))){
        		 if(grid->getSpeed(commCells.at(i)) >grid->getSpeed(commCells.at(i+1))){
        			 modifiedCosts.push_back(baseUnit-grid->getSpeed(commCells.at(i+1)));
        		 }else{
        			 modifiedCosts.push_back(baseUnit-grid->getSpeed(commCells.at(i)));
        		 }
        	 }else{
        		 modifiedCosts.push_back(cost.at(i));
        	 }
         }
         modifiedCosts.push_back(0);
         std::vector<int> buffState;
         	double toReturn=0;
         	for(int i=0;i<commCells.size();i++)
         		buffState.push_back(0);

         	for(int j=0;j<commCells.size()-1;j++){
         		if((buffState.at(j)+modifiedCosts.at(j))>=0){
         		buffState.at(j+1)=buffState.at(j)+modifiedCosts.at(j);
         		}else{
         			buffState.at(j+1)=0;
         		}
         	}

         int currentPos=0;

         while(needToUpload(buffState)){
        	 int toUp=0;
        	 int z=currentPos;
        	 for(z=currentPos;z<commCells.size();z++){
        		 if(grid->getSpeed(commCells.at(z))>grid->getSpeed(commCells.at(currentPos))){
        			 break;
        		 }
        	 }
        	 if(z>=commCells.size()-1){
        		 toUp=buffState.at(currentPos);
        		 for(int k=currentPos;k<commCells.size();k++){
        			 if(modifiedCosts.at(k)<0&&calculatePathCost(modifiedCosts,currentPos,k)<=buffer*baseUnit){
        				 toUp=toUp+modifiedCosts.at(k);
        			 }else{
        				 if(calculatePathCost(modifiedCosts,currentPos,k)>buffer*baseUnit){
        					 break;
        				 }
        			 }
        		 }
        		 if(toUp>0){
        		 toReturn=toReturn+ toUp/((double)grid->getSpeed(commCells.at(currentPos))-baseUnit);
        		 updateBufferStates(&buffState,toUp,currentPos);
        		 }
        		 currentPos=currentPos+1;

        	 }else{
        		 if(buffState.at(currentPos)+calculatePathCost(modifiedCosts,currentPos,z)>buffer*baseUnit){
        		  	  toUp=buffState.at(currentPos)+calculatePathCost(modifiedCosts,currentPos,z)-buffer*baseUnit;
        		  	  if(toUp>0){
        		  		  toReturn=toReturn+toUp/(double)(grid->getSpeed(commCells.at(currentPos))-baseUnit);
        		  		  updateBufferStates(&buffState,toUp,currentPos);
        		  	  }
        	 	 }
        	 	 currentPos=z;
        	 }
         }
         return toReturn;


}
bool Baseline::needToUpload(std::vector<int> bufferStates){
	for(int i=0;i<bufferStates.size();i++){
		if(bufferStates.at(i)>baseUnit*buffer){
			return true;
		}
	}
	return false;
}

void Baseline::updateBufferStates(std::vector<int>* buffStates,int uploaded,int at){
	for(int i=at;i<buffStates->size();i++){
		if(buffStates->at(i)-uploaded>=0){
			buffStates->at(i)=buffStates->at(i)-uploaded;
		}else{
			buffStates->at(i)=0;
		}
	}

}
int Baseline::calculatePathCost(std::vector<int> path,int beg,int end){
	int c=0;
	for(int i=beg;i<end;i++){
		c=c+path.at(i);
	}
	return c;
}






} /* namespace planner */
