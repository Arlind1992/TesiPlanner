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
#include <lemon/lgf_writer.h>
#include <lemon/dim2.h>
#include <lemon/lgf_reader.h>
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

//TODO write in file check if the graph is connected
bool planner::ComplexPlanner::makePlan(Cell cgoal,int Tmax,Cell cinit
		,vector<Cell>& result){
/*
	std::vector<Cell> toCheck;
	toCheck.push_back(cinit);
	std::vector<Cell> checked;
	DiGraph::Node node=graph.addNode();
	cellNode[cinit]=node;
	nodePoint[node].x=cinit.first;
	nodePoint[node].y=cinit.second;
	while(!toCheck.empty()){
		Cell cell=toCheck.front();
		toCheck.erase(toCheck.begin());
		checked.push_back(cell);
		std::vector<Cell> comm_cells=grid->commCells(cell,Tmax,cgoal);
		std::vector<Cell>::iterator it;
		for(it=comm_cells.begin();it<comm_cells.end();it++){
			if(!searchCell(checked,*it)){
				std::vector<Cell> pathTo;
				planner.makePlan(cell,*it,pathTo);
				double distance=this->calculateNum((grid->pathCost(pathTo)));
				if(distance<=Tmax){
					try{
						node=cellNode.at(*it);
						}catch(const std::out_of_range& oor){
						node=graph.addNode();
						cellNode[*it]=node;
						nodePoint[node].x=(*it).first;
						nodePoint[node].y=(*it).second;
					}

						 if(!searchCell(toCheck,*it)){
										    	toCheck.push_back(*it);
						 }
						 DiGraph::Node toconnect=cellNode[cell];
						 DiGraph::Arc arc=graph.addArc(toconnect,node);
						 length[arc]=distance;
						 DiGraph::Arc inverseArc=graph.addArc(node,toconnect);
						 length[inverseArc]=distance;
						}
			}
		}
		comm_cells.clear();

	}
	try{
	cellNode.at(cgoal);
	}catch(const std::out_of_range& oor){
		return false;
	}*/

	this->connectCell(cgoal);
	this->connectCell(cinit);
	this->connectCells(cgoal,cinit);
	this->connectFirstCellComplex(cinit);
	this->connectGoalCellComplex(cgoal);
	this->connectComplexCells(cinit,cgoal);
	lemon::Dijkstra<DiGraph,DiGraph::ArcMap<double>> solver(this->complexCaseGraph,this->lengthComplex);
	DiGraph::Node firstCompl;
	DiGraph::Node endCompl;
	firstCompl=nodeToVec[cellNode[cinit]].at(0);
	endCompl=nodeToVec[cellNode[cgoal]].at(0);
	solver.run(firstCompl,endCompl);
	if(complexCaseGraph.id(solver.predNode(endCompl))==-1){
		return false;
	}
	 std::vector<Cell> vec;
	 for (DiGraph::Node v=endCompl;v != firstCompl; v=solver.predNode(v)) {
		 DiGraph::Node prevNode=solver.predNode(v);
		 std::cout<<graph.id(prevNode)<<std::endl;
      DiGraph::Arc arc= lemon::findArc(complexCaseGraph,prevNode,v,lemon::INVALID);
      if(this->isMoving[arc]){
    	  Cell endCell;
		  lemon::dim2::Point<int> p=nodePoint[this->complexToNormal[v]];
		  endCell.first=p.x;
		  endCell.second=p.y;
    	  Cell startCell;
		  lemon::dim2::Point<int> p2=nodePoint[this->complexToNormal[prevNode]];
    	  startCell.first=p2.x;
    	  startCell.second=p2.y;
    	  this->planner.makePlan(startCell,endCell,vec);
    	  reverse(vec.begin(),vec.end());
    	  result.insert(result.end(),vec.begin(),vec.end());
    	  vec.clear();

      }
      }
    reverse(result.begin(),result.end());
    result.erase(unique(result.begin(),result.end()),result.end());
	return true;
}



bool planner::ComplexPlanner::searchCell(std::vector<Cell> cells,Cell toSearch){
	for(Cell cell :cells){
		if(cell.first==toSearch.first&&cell.second==toSearch.second){
			return true;
		}
	}
	return false;
}

void planner::ComplexPlanner::createComplexGraph(){
	createComplexNodes();
	std::cout<<"creation"<<std::endl;
	createArcsSameNodes();
	std::cout<<"same node connection"<<std::endl;
	connectDifferentNodes();
	std::cout<<"different node connection"<<std::endl;

}

void planner::ComplexPlanner::connectFirstCellComplex(Cell cell){
	//first create the first and last nodes
				DiGraph::Node firstNode=this->cellNode[cell];
				DiGraph::Node firstComplexNode=complexCaseGraph.addNode();
				std::vector<DiGraph::Node> firstNodesVec;
				firstNodesVec.push_back(firstComplexNode);
				for(DiGraph::Node n:firstNodesVec){
					this->complexToNormal[n]=firstNode;
				}
				this->nodeToVec[firstNode]=firstNodesVec;
	//for loop to connect with the other nodes where the first node is not a transmition node
	for(DiGraph::OutArcIt out(graph,firstNode);out!=INVALID;++out){
		//next node in the normal graph connected
		DiGraph::Node nod=graph.target(out);
		if(grid->isComm(std::make_pair(this->nodePoint[nod].x,this->nodePoint[nod].y))){
		double distance=this->length[out];
		std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
		DiGraph::Arc addedArc=this->complexCaseGraph.addArc(firstComplexNode,vecEndNodes.at(distance/this->discretizationPar));
		this->lengthComplex[addedArc]=distance;
		this->isMoving[addedArc]=true;
		}
	}
}
void planner::ComplexPlanner::createComplexNodes(){

	   /*
		 * for loop that goes through all the nodes of the graph we want to transform
		 * checks if the cell associated with the node is a communication node
		 * then transforms it
		 */
		for(DiGraph::NodeIt n(graph); n != INVALID; ++n ){

			std::vector<DiGraph::Node> vecNodes;
			if(this->grid->isComm(std::make_pair(this->nodePoint[n].x,this->nodePoint[n].y))){
				for(int i=0;i<numberOfNodes;i++){
					DiGraph::Node addNode=this->complexCaseGraph.addNode();
					vecNodes.push_back(addNode);
				}
				this->nodeToVec[n]=vecNodes;

				for(DiGraph::Node no:vecNodes){
								this->complexToNormal[no]=n;
							}
				vecNodes.clear();
			}


		}
}
void planner::ComplexPlanner::createArcsSameNodes(){
	/*
		 * for loop that creates the connections between the nodes in the
		 * complex case
		 */
		for(DiGraph::NodeIt b(graph); b != INVALID; ++b ){
				int speed=this->grid->getSpeed(std::make_pair(this->nodePoint[b].x,this->nodePoint[b].y));
				if(speed!=0){
				std::vector<DiGraph::Node> vecStartNodes=nodeToVec[b];

				//for loops to create connections between comm nodes associated to same cell
				    for(int i=1;i<speed/this->discretizationPar;i++){
						DiGraph::Arc arc=this->complexCaseGraph.addArc(vecStartNodes.at(i),vecStartNodes.at(0));
						this->lengthComplex[arc]=this->discretizationPar;
						this->isMoving[arc]=false;
				    }
				    for(int i=0;i<numberOfNodes-speed/this->discretizationPar;i++){
				    	DiGraph::Arc toadd=this->complexCaseGraph.addArc(vecStartNodes.at(i+speed),vecStartNodes.at(i));
				    	this->lengthComplex[toadd]=this->discretizationPar;
				    	this->isMoving[toadd]=false;
				    }
				}
		}

}

void planner::ComplexPlanner::connectGoalCellComplex(Cell end ){
	DiGraph::Node endNode=this->cellNode[end];
			DiGraph::Node endComplexNode=complexCaseGraph.addNode();
			std::vector<DiGraph::Node> endNodesVec;
			endNodesVec.push_back(endComplexNode);
			for(DiGraph::Node n:endNodesVec){
						this->complexToNormal[n]=endNode;
					}
			this->nodeToVec[endNode]=endNodesVec;
	//add end node in the graph
		for(DiGraph::InArcIt in(graph,endNode);in!=INVALID;++in){
				//next node in the normal graph connected
				DiGraph::Node nod=graph.source(in);
				if(grid->isComm(std::make_pair(this->nodePoint[nod].x,this->nodePoint[nod].y))){

				double distance=this->length[in];
				std::vector<DiGraph::Node> vecSourceNodes=nodeToVec[nod];
				for(int j=0;j<1+(TMAX-distance)/this->discretizationPar;j++){
					DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecSourceNodes.at(j),endComplexNode);
					this->lengthComplex[addedArc]=distance;
					this->isMoving[addedArc]=true;
				}

			}
		}
}


void planner::ComplexPlanner::connectDifferentNodes(){
		/*
		 * for loop that creates the connections between the nodes in the
		 * complex case
		 */
		for(DiGraph::NodeIt b(graph); b != INVALID; ++b ){
				if(this->grid->isComm(std::make_pair(this->nodePoint[b].x,this->nodePoint[b].y))){
				std::vector<DiGraph::Node> vecStartNodes=nodeToVec[b];
				    //for loop to connect with the other nodes
					for(DiGraph::OutArcIt out(graph,b);out!=INVALID;++out){
						//next node in the normal graph connected
						DiGraph::Node nod=graph.target(out);
						if(!this->grid->isComm(std::make_pair(this->nodePoint[nod].x,this->nodePoint[nod].y)))
							continue;
						double distance=this->length[out];

						std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
						for(int j=0;j<numberOfNodes-distance/this->discretizationPar;j++){
							DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecStartNodes.at(j),vecEndNodes.at(j+distance/this->discretizationPar));
							this->lengthComplex[addedArc]=distance;
							this->isMoving[addedArc]=true;
						}
					}
				}
			}
}

double	planner::ComplexPlanner::calculateNum(double distance){
	return ceil(distance/this->discretizationPar)*this->discretizationPar;
}

void planner::ComplexPlanner::createNormalGraph(){
	createNodes();
	createCellNode();
	connectNormalNodes();
}
void planner::ComplexPlanner::createNodes(){
	for(int i=0;i<grid->getMaxX();i++){
		for(int j=0;j<grid->getMaxY();j++){
			if(this->grid->isComm(std::make_pair(i,j))&&this->grid->isFree(std::make_pair(i,j))){
				DiGraph::Node addedNode=graph.addNode();
				lemon::dim2::Point<int> p(i,j);
				this->nodePoint[addedNode]=p;
			}
		}
	}
}

void planner::ComplexPlanner::createCellNode(){
	for(DiGraph::NodeIt n(graph);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		this->cellNode[std::make_pair(p.x,p.y)]=n;
	}
}
void planner::ComplexPlanner::connectNormalNodes(){
	for(DiGraph::NodeIt n(graph);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		std::vector<Cell> comm=this->grid->getCommCells(std::make_pair(p.x,p.y),TMAX);
		for(Cell c:comm){
			std::vector<Cell> path;
			if(this->planner.makePlan(std::make_pair(p.x,p.y),c,path)){
				double distance=this->calculateNum(this->grid->pathCost(path));
				if(distance<=TMAX){
					DiGraph::Node toConnect=cellNode[c];
					DiGraph::Arc arc=graph.addArc(n,toConnect);
					this->length[arc]=distance;
				}
				path.clear();
			}

		}
		comm.clear();
	}
}

void planner::ComplexPlanner::connectCell(Cell cell){
	DiGraph::Node addedNode=graph.addNode();
	lemon::dim2::Point<int> p(cell.first,cell.second);
	this->nodePoint[addedNode]=p;
	this->cellNode[cell]=addedNode;
	std::vector<Cell> comm=this->grid->getCommCells(cell,TMAX);
			for(Cell c:comm){
				std::vector<Cell> path;
				if(this->planner.makePlan(cell,c,path)){
					double distance=this->calculateNum(this->grid->pathCost(path));
					if(distance<=TMAX){
						DiGraph::Node toConnect=cellNode[c];
						DiGraph::Arc arc=graph.addArc(addedNode,toConnect);
						DiGraph::Arc reverseArc=graph.addArc(toConnect,addedNode);
						this->length[arc]=distance;
						this->length[reverseArc]=distance;
					}
				}
				path.clear();
			}
}
void planner::ComplexPlanner::connectCells(Cell cell1,Cell cell2){
	std::vector<Cell> path;
	if(this->planner.makePlan(cell1,cell2,path)){
		double distance=this->calculateNum(this->grid->pathCost(path));
		if(distance<=TMAX){
			DiGraph::Node node1=cellNode[cell1];
			DiGraph::Node node2=cellNode[cell2];
			DiGraph::Arc arc=graph.addArc(node1,node2);
			DiGraph::Arc ar1=graph.addArc(node2,node1);
			this->length[arc]=distance;
			this->length[ar1]=distance;
		}
	}
}
void planner::ComplexPlanner::connectComplexCells(Cell start,Cell goal){
		if(lemon::findArc(graph,cellNode[start],cellNode[goal],lemon::INVALID)!=lemon::INVALID){
			DiGraph::Arc arc=complexCaseGraph.addArc(this->nodeToVec[cellNode[start]].at(0),this->nodeToVec[cellNode[goal]].at(0));
			this->lengthComplex[arc]=this->length[lemon::findArc(graph,cellNode[start],cellNode[goal],lemon::INVALID)];
			this->isMoving[arc]=true;
		}
}

void planner::ComplexPlanner::createGraphs(){
	if(FILE *file = fopen(this->filePath, "r")) {
		fclose(file);
			DigraphReader<DiGraph>( graph,this->filePath).nodeMap("Point",this->nodePoint).arcMap("length",this->length).run();
			this->createCellNode();
		}else{
			this->createNormalGraph();
			DigraphWriter<DiGraph>(graph, this->filePath).nodeMap("Point",this->nodePoint).arcMap("length",this->length).run();
	}
	this->createComplexGraph();

}
void planner::ComplexPlanner::setFilePath(char* filePath){
	this->filePath=filePath;
}



} /* namespace planner */
