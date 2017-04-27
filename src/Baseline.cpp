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
			 toCalculateT.push_back(startCell);
			 costForT.push_back(buffCost);
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
	    if(std::isinf(transmittionTime)||transmittionTime<0){
	       	(*myfile)<<"InfiniteCase Cell ("<<cinit.first<<","<<cinit.second<<")-("<<cgoal.first<<","<<cgoal.second<<")"<<std::endl;
	    }
		(*myfile)<<"Baseline computational cost: "<<((stop_s-start_s)/double(CLOCKS_PER_SEC)*1000) <<"\n";
		(*myfile)<<"Baseline transmition cost : "<<transmittionTime<<"\n";
		(*myfile)<<"Baseline path cost : "<<movingTime<<"\n";
		return true;
}

void planner::Baseline::createGraph(){

	int s1=clock();
	std::cout<<"change"<<std::endl;
	this->createNodes();
	this->createCellNode();
	this->connectNodes();
	int s2=clock();
	(*myfile)<<"Time to create second graph (nodes with bigger Rate): "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<"\n";

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

//
double Baseline::calculateTime(std::vector<Cell> commCells,std::vector<int> cost){
	std::vector<int> buffState;
	double toReturn=0;
	for(int i=0;i<commCells.size();i++)
		buffState.push_back(0);

	for(int j=0;j<commCells.size()-1;j++){
		buffState.at(j+1)=buffState.at(j)+cost.at(j);
	}
	for(int z=0;z<commCells.size();z++){
		if(buffState.at(z)>buffer*baseUnit){
			int toUpload=buffState.at(z)-buffer*baseUnit;
			int k;
			int sum=0;
			for(k=z-1;k>0;k--){
				sum=sum+cost.at(k);
				if(sum>buffer*baseUnit)
					break;
			}

			std::vector<int> alreadyUp;
			while(toUpload!=0){
				int b;
				for(b=k;b<z;b++){
					if(buffState.at(b)!=0)
						break;
				}
				int fastCell=b;
				for(int g=b;g<z;g++){
					if(grid->getSpeed(commCells.at(g))>grid->getSpeed(commCells.at(fastCell))&&buffState.at(g)!=0){
						fastCell=g;
					}
				}
				int toSubtract;
				if(toUpload>buffState.at(fastCell)){
					toSubtract=buffState.at(fastCell);
				}else{
					toSubtract=toUpload;
				}
				toReturn=toReturn+toSubtract/((double)grid->getSpeed(commCells.at(fastCell))-baseUnit);
				toUpload=toUpload-toSubtract;
				for(int d=fastCell;d<commCells.size();d++){
					buffState.at(d)=buffState.at(d)-toSubtract;
				}

			}

		}
	}
	std::cout<<"buffer state"<<std::endl;
	for(int bmn=0;bmn<buffState.size();bmn++){
		std::cout<<buffState.at(bmn)<<std::endl;
	}
	std::cout<<"transmission time "<< toReturn<<std::endl;
	std::cout<<"end State"<<std::endl;

	return toReturn;

}


} /* namespace planner */
