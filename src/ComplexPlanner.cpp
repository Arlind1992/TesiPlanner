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
#include <typeinfo>
using namespace std;
using namespace rrt_planning;
using namespace lemon;

namespace planner {


ComplexPlanner::~ComplexPlanner() {
	// TODO Auto-generated destructor stub
}
bool planner::ComplexPlanner::makePlan(Cell cgoal,Cell cinit
		,vector<Cell>& result){
	int start_s=clock();
	bool isCommInit=grid->isComm(cinit);
	bool isCommGoal=grid->isComm(cgoal);
	movingTime=0;
	transmittionTime=0;
	if(!grid->isFree(cgoal)||!grid->isFree(cinit)){
		return false;
	}

	if(!isCommInit)
		this->connectCell(cinit);
	if(!isCommGoal)
		this->connectCell(cgoal);
	if(!isCommInit&&!isCommGoal)
		this->connectCells(cgoal,cinit);
	if(!isCommInit)
		this->connectFirstCellComplex(cinit);
	if(!isCommGoal)
		this->connectGoalCellComplex(cgoal);
	if(!isCommGoal&&!isCommInit)
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
    	  this->planner->makePlan(startCell,endCell,vec,buffer);
    	  reverse(vec.begin(),vec.end());
    	  result.insert(result.end(),vec.begin(),vec.end());
    	  vec.clear();
    	  movingTime=movingTime+this->lengthComplex[arc];
      }else{
    	  lemon::dim2::Point<int> p=this->nodePoint[this->complexToNormal[prevNode]];
    	  std::cout<<"speed"<<grid->getSpeed(std::make_pair(p.x,p.y))<<std::endl;
    	  transmittionTime=transmittionTime+this->discretizationPar;
      }
      }
	reverse(result.begin(),result.end());
    result.erase(unique(result.begin(),result.end()),result.end());
    int stop_s=clock();
    if(!grid->isComm(cinit)){
    	graph.erase(cellNode[cinit]);
    	complexCaseGraph.erase(firstCompl);
    	cellNode.erase(cinit);
    }
    if(!grid->isComm(cgoal)){
    	graph.erase(cellNode[cgoal]);
    	complexCaseGraph.erase(endCompl);
    	cellNode.erase(cgoal);
    }
	this->myfile<<"Complex computational time: "<<(stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 <<"\n";
	myfile<<"Complex transmittion cost : "<<transmittionTime<<"\n";
	myfile<<"Complex path cost : "<<movingTime<<"\n";
	 std::cout<<"transCost "<<transmittionTime<<"mls"<<std::endl;
		    std::cout<<"path cost"<<movingTime<<std::endl;
	return true;
}

bool planner::ComplexPlanner::makeSimplePlan(Cell cgoal,Cell cinit,std::vector<Cell>& result){
	int start_s=clock();
	if(!grid->isFree(cgoal)||!grid->isFree(cinit)){
			return false;
		}
	double computationalTime=0;
	movingTime=0;
		transmittionTime=0;
		bool isCommInit=grid->isComm(cinit);
			bool isCommGoal=grid->isComm(cgoal);
	if(!isCommInit)
			this->connectCell(cinit);
		if(!isCommGoal)
			this->connectCell(cgoal);
		if(!isCommInit&&!isCommGoal)
			this->connectCells(cgoal,cinit);
		lemon::Dijkstra<DiGraph,DiGraph::ArcMap<double>> solver(graph,length);

		DiGraph::Node first=cellNode[cinit];
		DiGraph::Node end=cellNode[cgoal];
		solver.run(first,end);
		if(graph.id(solver.predNode(end))==-1){
			return false;
		}
		 std::vector<Cell> vec;
		 for (DiGraph::Node v=end;v != first; v=solver.predNode(v)) {
		 DiGraph::Node prevNode=solver.predNode(v);
		 Cell endCell;
		 lemon::dim2::Point<int> p=nodePoint[v];
		 endCell.first=p.x;
		 endCell.second=p.y;
		 Cell startCell;
		 lemon::dim2::Point<int> p2=nodePoint[prevNode];
    	  startCell.first=p2.x;
    	  startCell.second=p2.y;
    	  this->planner->makePlan(startCell,endCell,vec,buffer);
    	  int s1=clock();

    	  if(prevNode!=first){
    		  int distanceTill=solver.dist(prevNode);
    		  int vecCost=this->grid->pathCost(vec);
    		  if(this->grid->isComm(startCell)){
    		  if(distanceTill+vecCost>buffer){
    			  if(distanceTill>=buffer){
    				  transmittionTime=transmittionTime+ceil(vecCost/this->grid->getSpeed(startCell));
    			  }else{
    				  transmittionTime=transmittionTime +ceil((vecCost+distanceTill-buffer)/this->grid->getSpeed(startCell));
    			  }

    		  }
    		  }
    	  }
    	  int s2=clock();
    	  computationalTime=computationalTime+(s2-s1)/double(CLOCKS_PER_SEC)*1000;
    	  reverse(vec.begin(),vec.end());
    	  result.insert(result.end(),vec.begin(),vec.end());
    	  vec.clear();
	      }
	    reverse(result.begin(),result.end());
	    result.erase(unique(result.begin(),result.end()),result.end());
	    int stop_s=clock();

	    double movingTime=solver.dist(cellNode[cgoal]);
	    //calculate the virtual time the robot spends sending information and moving throught the grid
	    std::cout<<"BaseLine "<<transmittionTime<<"mls"<<std::endl;
	    std::cout<<"path cost"<<movingTime<<std::endl;

	    if(!grid->isComm(cinit)){
	        	graph.erase(cellNode[cinit]);
	        	cellNode.erase(cinit);
	        }
	        if(!grid->isComm(cgoal)){
	        	graph.erase(cellNode[cgoal]);
	        	cellNode.erase(cgoal);
	        }
		this->myfile<<"Normal computational cost: "<<((stop_s-start_s)/double(CLOCKS_PER_SEC)*1000)-computationalTime <<"\n";
		this->myfile<<"Baseline computational cost: "<<((stop_s-start_s)/double(CLOCKS_PER_SEC)*1000) <<"\n";

		myfile<<"Normal path cost : "<<movingTime<<"\n";
		myfile<<"Baseline transmition cost : "<<transmittionTime<<"\n";
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

	int start_s=clock();
	createComplexNodes();
	createArcsSameNodes();
	connectDifferentNodes();
	int stop_s=clock();
	this->myfile<<"Time to create complex graph "<<(stop_s-start_s)/double(CLOCKS_PER_SEC)*1000<<"mls with a discretization parameter "<<this->discretizationPar<<std::endl;

}

void planner::ComplexPlanner::connectFirstCellComplex(Cell cell){
	//first create the first nodes
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
		double distance=calculateNum(this->length[out]);
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
void planner::ComplexPlanner::createArcsSameNodes(){
	/*
		 * for loop that creates the connections between the nodes in the
		 * complex case
		 */
		for(DiGraph::NodeIt b(graph); b != INVALID; ++b ){
				int speed=this->grid->getSpeed(std::make_pair(this->nodePoint[b].x,this->nodePoint[b].y));
					std::vector<DiGraph::Node> vecStartNodes=nodeToVec[b];

				//for loops to create connections between comm nodes associated to same cell
				    for(int i=1;i<calculateNum(speed/this->discretizationPar);i++){
						DiGraph::Arc arc=this->complexCaseGraph.addArc(vecStartNodes.at(i),vecStartNodes.at(0));
						this->lengthComplex[arc]=this->discretizationPar;
						this->isMoving[arc]=false;
				    }
				    for(int i=0;i<calculateNum(numberOfNodes-speed/this->discretizationPar);i++){
				    	DiGraph::Arc toadd=this->complexCaseGraph.addArc(vecStartNodes.at(i+speed),vecStartNodes.at(i));
				    	this->lengthComplex[toadd]=this->discretizationPar;
				    	this->isMoving[toadd]=false;
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

				double distance=calculateNum(this->length[in]);
				std::vector<DiGraph::Node> vecSourceNodes=nodeToVec[nod];
				for(int j=0;j<1+calculateNum((buffer-distance)/this->discretizationPar);j++){
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
				std::vector<DiGraph::Node> vecStartNodes=nodeToVec[b];
				    //for loop to connect with the other nodes
					for(DiGraph::OutArcIt out(graph,b);out!=INVALID;++out){
						//next node in the normal graph connected
						DiGraph::Node nod=graph.target(out);
						double distance=calculateNum(this->length[out]);

						std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
						for(int j=0;j<numberOfNodes-distance/this->discretizationPar;j++){
							DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecStartNodes.at(j),vecEndNodes.at(j+distance/this->discretizationPar));
							this->lengthComplex[addedArc]=distance;
							this->isMoving[addedArc]=true;
						}
					}
			}
}

double	planner::ComplexPlanner::calculateNum(double distance){
	return ceil(distance/this->discretizationPar)*this->discretizationPar;
}
/*
 * Normal graph calculated using Theta* to calculate the distance between communicatin
 * nodes
 */
void planner::ComplexPlanner::createNormalGraph(){
	createNodes();
	createCellNode();
	connectNormalNodes();

}
void planner::ComplexPlanner::createNodes(){
	for(int i=0;i<grid->getMaxX();i++){
		for(int j=0;j<grid->getMaxY();j++){
			if(this->grid->isComm(std::make_pair(i,j)) && this->grid->isFree(std::make_pair(i,j))){
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
	//TODO change
	std::vector<Cell> checked;
	/*int i=0;
	for(DiGraph::NodeIt n(graph);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		std::vector<Cell> comm=this->grid->getCommCells(std::make_pair(p.x,p.y),buffer);
		checked.push_back(std::make_pair(p.x,p.y));
		for(Cell c:comm){
			i++;
			std::vector<Cell> path;
			if(this->searchCell(checked,c)){
				int s3=clock();
				if(this->planner->makePlan(std::make_pair(p.x,p.y),c,path,buffer)){
					double distance=this->grid->pathCost(path);
					if(distance<=buffer){
						DiGraph::Node toConnect=cellNode[c];
						DiGraph::Arc arc=graph.addArc(n,toConnect);
						DiGraph::Arc reverseArc=graph.addArc(toConnect,n);
						this->length[arc]=distance;
						this->length[reverseArc]=distance;
					}
					path.clear();
				}

				int s4=clock();
				std::cout<<i<<" : "<<(s4-s3)/double(CLOCKS_PER_SEC)*1000<<"\n";


			}
		}
		comm.clear();
	}*/
	for(DiGraph::NodeIt n(graph);n!=INVALID;++n){
		lemon::dim2::Point<int> p=nodePoint[n];
		std::vector<Cell> cells;
		std::vector<int> distances;
		this->planner->makePlan(std::make_pair(p.x,p.y),cells,buffer,distances);
		for(int i=0;i<cells.size();i++){
			DiGraph::Node toConnect=this->cellNode[cells.at(i)];
			DiGraph::Arc conn=graph.addArc(n,toConnect);
			length[conn]=distances.at(i);
		}
		cells.clear();
		distances.clear();


	}
	//std::cout<<"times :"<<i<<std::endl;
}

void planner::ComplexPlanner::connectCell(Cell cell){
	DiGraph::Node addedNode=graph.addNode();
	lemon::dim2::Point<int> p(cell.first,cell.second);
	this->nodePoint[addedNode]=p;
	this->cellNode[cell]=addedNode;
	std::vector<Cell> comm=this->grid->getCommCells(cell,buffer);
			for(Cell c:comm){
				std::vector<Cell> path;
				if(this->planner->makePlan(cell,c,path,buffer)){
					double distance=this->grid->pathCost(path);
					if(distance<=buffer){
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
	if(this->planner->makePlan(cell1,cell2,path,buffer)){
		double distance=this->grid->pathCost(path);
		if(distance<=buffer){
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
			this->lengthComplex[arc]=this->calculateNum(this->length[lemon::findArc(graph,cellNode[start],cellNode[goal],lemon::INVALID)]);
			this->isMoving[arc]=true;
		}
}

void planner::ComplexPlanner::createGraphs(){
	if(FILE *file = fopen(filePath, "r")) {

		fclose(file);
			DigraphReader<DiGraph>( graph,filePath).nodeMap("Point",this->nodePoint).arcMap("length",this->length).run();
			this->createCellNode();
		}else{
			 int s1=clock();
			this->createNormalGraph();
			int s2=clock();
			std::cout<<"time to create normal Graph "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<std::endl;
			this->myfile<<"time to create normal Graph "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<"\n";
			DigraphWriter<DiGraph>(graph, filePath).nodeMap("Point",this->nodePoint).arcMap("length",this->length).run();
	}
	int s1=clock();
	this->createComplexGraph();
	int s2=clock();
	myfile<<"Time to create Complex graph : "<<(s2-s1)/double(CLOCKS_PER_SEC)*1000<<"\n";

}

} /* namespace planner */
