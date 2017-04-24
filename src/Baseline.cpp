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
#include <lemon/lgf_writer.h>
#include <lemon/dim2.h>
#include <lemon/lgf_reader.h>
#include <typeinfo>
using namespace std;
using namespace rrt_planning;
using namespace lemon;
namespace planner {

Baseline::~Baseline() {
	// TODO Auto-generated destructor stub
}
bool planner::Baseline::makePlan(Cell cgoal,Cell cinit
		,vector<Cell>& result ){
	int start_s=clock();
		if(!grid->isFree(cgoal)||!grid->isFree(cinit)){
				return false;
			}
		movingTime=0;
			transmittionTime=0;
			bool isCommInit=grid->isComm(cinit);
				bool isCommGoal=grid->isComm(cgoal);
		if(!isCommInit)
				this->connectCellBiggerRate(cinit);
			if(!isCommGoal)
				this->connectCellBiggerRate(cgoal);
			if(!isCommInit&&!isCommGoal)
				this->connectBiggerRateCells(cgoal,cinit);
			lemon::Dijkstra<BaGraph,BaGraph::ArcMap<int>> solver(graphNodesBiggerRate,lengthBiggerRate);

			BaGraph::Node first=cellBiggerRateNodes[cinit];
			BaGraph::Node end=cellBiggerRateNodes[cgoal];
			std::vector<Cell> toCalculateT;
			std::vector<int> costForT;
			toCalculateT.push_back(cgoal);
			costForT.push_back(0);
			solver.run(first,end);
			if(graphNodesBiggerRate.id(solver.predNode(end))==-1){
				return false;
			}
			 std::vector<Cell> vec;
			 for (BaGraph::Node v=end;v != first; v=solver.predNode(v)) {
			 BaGraph::Node prevNode=solver.predNode(v);
			 Cell endCell;
			 lemon::dim2::Point<int> p=biggerRateNodePoint[v];
			 endCell.first=p.x;
			 endCell.second=p.y;
			 Cell startCell;
			 lemon::dim2::Point<int> p2=biggerRateNodePoint[prevNode];
	    	  startCell.first=p2.x;
	    	  startCell.second=p2.y;
	    	  int cost;
	    	  makePlanAllNodes(startCell,endCell,vec,&cost);
	    	  toCalculateT.push_back(startCell);
	    	  costForT.push_back(cost);
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

		    movingTime=solver.dist(cellBiggerRateNodes[cgoal]);

		    if(!grid->isComm(cinit)){
		    	cellBiggerRateNodes.erase(cinit);
		        }
		        if(!grid->isComm(cgoal)){
		        	cellBiggerRateNodes.erase(cgoal);
		        }
			(*myfile)<<"Baseline computational cost: "<<((stop_s-start_s)/double(CLOCKS_PER_SEC)*1000) <<"\n";
			(*myfile)<<"Baseline transmition cost : "<<transmittionTime<<"\n";
			return true;
}

void Baseline::makePlanAllNodes(Cell start,std::vector<Cell>& cells,int buffer,std::vector<int>& distances){
	BaGraph::Node firstNode=cellAllNodes[start];
	lemon::Dijkstra<BaGraph,BaGraph::ArcMap<int> > solver(this->graphAllNodes,this->length);
		solver.run(firstNode);
	for(BaGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
		//std::cout<<"after"<<std::endl;
		lemon::dim2::Point<int> p=allNodePoint[n];
		if(!grid->isComm(std::make_pair(p.x,p.y)))
			continue;
		if(solver.dist(n)<=buffer*baseUnit){
			cells.push_back(std::make_pair(p.x,p.y));
			int count=0;
			for (BaGraph::Node v=n;v != firstNode; v=solver.predNode(v)) {
				count++;
			}
			distances.push_back(count);
		}
	}
}
bool Baseline::makePlanAllNodes(Cell start,Cell end,std::vector<Cell> &result,int *cost){
	BaGraph::Node firstNode=this->cellAllNodes[start];
		BaGraph::Node endNode=this->cellAllNodes[end];
		lemon::Dijkstra<BaGraph,BaGraph::ArcMap<int> > solver(this->graphAllNodes,this->length);
		solver.run(firstNode,endNode);
		if(graphAllNodes.id(solver.predNode(endNode))==-1){
				return false;
			}
		for (BaGraph::Node v=endNode;v != firstNode; v=solver.predNode(v)) {
			lemon::dim2::Point<int> p=allNodePoint[v];
			result.push_back(std::make_pair(p.x,p.y));
		}
		*cost=solver.dist(endNode);
		result.push_back(start);
		reverse(result.begin(),result.end());
		return true;

}






bool planner::Baseline::searchCell(std::vector<Cell> cells,Cell toSearch){
	for(Cell cell :cells){
		if(cell.first==toSearch.first&&cell.second==toSearch.second){
			return true;
		}
	}
	return false;
}
//DONE
void planner::Baseline::createBiggerRateGraph(){
	//DONE
	this->createBiggerRateNodes();
	//DONE
	this->createCellNodeBiggerRate();
	//DONE
	this->connectBiggerRateNodes();

}
//DONE
void planner::Baseline::createCellNodeBiggerRate(){
	for(BaGraph::NodeIt n(graphNodesBiggerRate);n!=INVALID;++n){
			lemon::dim2::Point<int> p=biggerRateNodePoint[n];
			this->cellBiggerRateNodes[std::make_pair(p.x,p.y)]=n;
		}
}



//DONE
void planner::Baseline::connectCellBiggerRate(Cell cell){
	BaGraph::Node addedNode=graphNodesBiggerRate.addNode();
				lemon::dim2::Point<int> p(cell.first,cell.second);
				this->biggerRateNodePoint[addedNode]=p;
				this->cellBiggerRateNodes[cell]=addedNode;
				std::vector<Cell> cells;
				std::vector<int> distances;
				makePlanAllNodes(cell,cells,buffer,distances);
				std::cout<<"size "<<cells.size()<<std::endl;
				for(int i=0;i<cells.size();i++){
					if(!grid->isComm(cells.at(i))||grid->getSpeed(cells.at(i))<=baseUnit){
						continue;
					}

					BaGraph::Node toConnect=this->cellBiggerRateNodes[cells.at(i)];
					BaGraph::Arc conn=graphNodesBiggerRate.addArc(addedNode,toConnect);
					BaGraph::Arc inv=graphNodesBiggerRate.addArc(toConnect,addedNode);
					lengthBiggerRate[conn]=distances.at(i);
					lengthBiggerRate[inv]=distances.at(i);
				}
				cells.clear();
				distances.clear();
}
//Done
void planner::Baseline::createBiggerRateNodes(){
		for(int i=0;i<grid->getMaxX();i++){
			for(int j=0;j<grid->getMaxY();j++){
				if(this->grid->isComm(std::make_pair(i,j)) && this->grid->isFree(std::make_pair(i,j))&&this->grid->getSpeed(std::make_pair(i,j))>baseUnit){
					BaGraph::Node addedNode=this->graphNodesBiggerRate.addNode();
					lemon::dim2::Point<int> p(i,j);
					this->biggerRateNodePoint[addedNode]=p;
				}
			}
		}

}

//DONE
void planner::Baseline::connectBiggerRateNodes(){
	for(BaGraph::NodeIt n(graphNodesBiggerRate);n!=INVALID;++n){
			lemon::dim2::Point<int> p=biggerRateNodePoint[n];
			std::vector<Cell> cells;
			std::vector<int> distances;
			makePlanAllNodes(std::make_pair(p.x,p.y),cells,buffer,distances);
			for(int i=0;i<cells.size();i++){
				if(grid->getSpeed(cells.at(i))<=baseUnit)
					continue;
				BaGraph::Node toConnect=this->cellBiggerRateNodes[cells.at(i)];
				BaGraph::Arc conn=graphNodesBiggerRate.addArc(n,toConnect);
				lengthBiggerRate[conn]=distances.at(i);

			}
			cells.clear();
			distances.clear();

		}


}

/*
 * Done
 */
void planner::Baseline::createAllNodesGraph(){
	createNodes();
	createCellNode();
	this->connectAllGraphNodes();

}
void planner::Baseline::createNodes(){
	for(int i=0;i<grid->getMaxX();i++){
		for(int j=0;j<grid->getMaxY();j++){
			if(this->grid->isFree(std::make_pair(i,j))){
				BaGraph::Node addedNode=this->graphAllNodes.addNode();
				lemon::dim2::Point<int> p(i,j);
				this->allNodePoint[addedNode]=p;
			}
		}
	}
}

void planner::Baseline::createCellNode(){
	for(BaGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
		lemon::dim2::Point<int> p=allNodePoint[n];
		this->cellAllNodes[std::make_pair(p.x,p.y)]=n;
	}
}
//DONE
void planner::Baseline::connectAllGraphNodes(){

		for(BaGraph::NodeIt n(graphAllNodes);n!=INVALID;++n){
			lemon::dim2::Point<int> p=allNodePoint[n];
			std::vector<Cell> neighbourCells=this->grid->getFourNeighbours(std::make_pair(p.x,p.y));
			for(Cell c:neighbourCells){
				BaGraph::Node toConnect=cellAllNodes[c];
				BaGraph::Arc addedEdge=graphAllNodes.addArc(n,toConnect);
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

void planner::Baseline::connectBiggerRateCells(Cell start,Cell goal){
		int cost;
		std::vector<Cell> sol;
		makePlanAllNodes(start,goal,sol,&cost);
		if(cost<=buffer*baseUnit){
			BaGraph::Arc arc=graphNodesBiggerRate.addArc(cellBiggerRateNodes[start],cellBiggerRateNodes[goal]);
			BaGraph::Arc invArc=graphNodesBiggerRate.addArc(cellBiggerRateNodes[goal],cellBiggerRateNodes[start]);
			this->lengthBiggerRate[invArc]=cost/baseUnit;
			this->lengthBiggerRate[arc]=cost/baseUnit;
		}
}

void planner::Baseline::createGraphs(){
	 int s4=clock();
	this->createAllNodesGraph();
	int s3=clock();
	std::cout<<"time to create first Graph "<<(s3-s4)/double(CLOCKS_PER_SEC)*1000<<std::endl;
	(*myfile)<<"time to create first Graph "<<(s3-s4)/double(CLOCKS_PER_SEC)*1000<<"\n";

	std::cout<<"Arcs = "<<lemon::countArcs(graphAllNodes)<<" gr "<<sizeof(BaGraph::Arc)<<std::endl;
	std::cout<<"Nodes = "<<lemon::countNodes(graphAllNodes)<<" gr "<<sizeof(BaGraph::Node)<<std::endl;
	int s1=clock();
	this->createBiggerRateGraph();
	std::cout<<"change"<<std::endl;
	std::cout<<"Arcs C= "<<lemon::countArcs(graphNodesBiggerRate)<<std::endl;
		std::cout<<"Nodes C= "<<lemon::countNodes(graphNodesBiggerRate)<<std::endl;

	int s2=clock();
	(*myfile)<<"Time to create second graph : "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<"\n";



}

int Baseline::findPosition(std::vector<BaGraph::Node> nodes,BaGraph::Node n){
	for(int i=0;i<(int)nodes.size();i++)
	{
		if(nodes.at(i)==n){
			return i;
		}
	}
	return -1;
}
//TODO
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
				sum=sum+cost.at(k-1);
				if(sum>buffer*baseUnit)
					break;
			}
			sum=0;
			std::vector<int> alreadyUp;
			while(toUpload!=0){
				int b;
				for(b=k;b<z;b++){
					if(buffState.at(b)!=0)
						break;
				}
				int fastCell=b;
				std::cout<<b<<" value of b"<<std::endl;
				for(int g=k;g<z;g++){
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
				toReturn=toReturn+toSubtract/(double)grid->getSpeed(commCells.at(fastCell));
				toUpload=toUpload-toSubtract;
				for(int d=fastCell;d<commCells.size();d++){
					buffState.at(d)=buffState.at(d)-toSubtract;
				}

			}

		}
	}
	for(int w=0;w<buffState.size();w++)
		std::cout<<"buffState "<<buffState.at(w)<<" transmiting time"<<toReturn<<std::endl;


	return toReturn;

}


} /* namespace planner */
