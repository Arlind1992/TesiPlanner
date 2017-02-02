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
				int distance=ceil(grid->pathCost(pathTo));
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
							 length[arc]=distance;
							 arcPath[arc]=pathTo;
							 DiGraph::Arc inverseArc=graph.addArc(node,toconnect);
							 length[inverseArc]=distance;
							 std::vector<rrt_planning::Cell> reversedPath=pathTo;
							 std::reverse(reversedPath.begin(),reversedPath.end());
							 arcPath[inverseArc]=reversedPath;
					}
			}
		}
		comm_cells.clear();

	}
	try{
	cellNode.at(cgoal);
	}catch(const std::out_of_range& oor){
		return false;
	}

	this->createComplexGraph(&cinit,&cgoal);
	lemon::Dijkstra<DiGraph,DiGraph::ArcMap<double>> solver(this->complexCaseGraph,this->lengthComplex);
	DiGraph::Node firstCompl;
	DiGraph::Node endCompl;

	try{
		firstCompl=nodeToVec.at(cellNode[cinit]).at(0);
		}catch(const std::out_of_range& oor){
			std::cout<<"problem beg"<<std::endl;
		}
		try{
			endCompl=nodeToVec.at(cellNode[cgoal]).at(0);
			}catch(const std::out_of_range& oor){
				std::cout<<"problem end"<<std::endl;
			}

	std::cout<<"id first:"<<complexCaseGraph.id(firstCompl)<<std::endl;

	std::cout<<"id last:"<<complexCaseGraph.id(endCompl)<<std::endl;
	solver.run(firstCompl,endCompl);
	std::cout<<"som"<<std::endl;
	//TODO delete after test part where stamp graph
	/*for(DiGraph::NodeIt n(graph); n != INVALID; ++n ){
				std::cout<<graph.id(n)<<std::endl;
				std::cout<<"representin:"<<"("<<this->nodeCell[n].first<<","<<this->nodeCell[n].second<<")"<<std::endl;
			}
	int k=0;
			for(DiGraph::ArcIt b(graph);b!=INVALID;++b){
				std::cout<<"Arc ="<<std::endl;
				std::cout<<"("<<nodeCell[(graph.source(b))].first<<","<<nodeCell[(graph.source(b))].second<<")"<<"-"<<"("<<nodeCell[(graph.target(b))].first<<","<<nodeCell[(graph.target(b))].second<<")"<<std::endl;

				std::cout<<"distance:"<<this->length[b]<<std::endl;
					std::cout<<"path:"<<std::endl;
					Planner::stampVector(arcPath[b]);
					std::cout<<graph.id(b)<<"-id"<<endl;
					std::cout<<"end Arc"<<std::endl;

					k++;

			}
			std::cout<<"number of arcs="<<k<<std::endl;
	std::cout<<"begin complex graph"<<endl;
*/
	for(DiGraph::NodeIt n(complexCaseGraph); n != INVALID; ++n ){
			std::cout<<complexCaseGraph.id(n)<<std::endl;
			std::cout<<"representin:"<<"("<<nodeCell[this->complexToNormal[n]].first<<","<<this->nodeCell[this->complexToNormal[n]].second<<")"<<std::endl;
		}
	/*
		for(DiGraph::ArcIt b(complexCaseGraph);b!=INVALID;++b){
			std::cout<<"("<<nodeCell[(complexCaseGraph.source(b))].first<<","<<nodeCell[(complexCaseGraph.source(b))].second<<")"<<"-"<<"("<<nodeCell[(complexCaseGraph.target(b))].first<<","<<nodeCell[(complexCaseGraph.target(b))].second<<")"<<std::endl;
				std::cout<<"distance:"<<this->lengthComplex[b]<<std::endl;
				std::cout<<"path:"<<std::endl;
				if(this->isMoving[b]){
				Planner::stampVector(arcPath[b]);
				}else{
					std::cout<<"communication"<<std::endl;
				}
		}*/
	 std::vector<rrt_planning::Cell> vec;
    for (DiGraph::Node v=endCompl;v != firstCompl; v=solver.predNode(v)) {
      DiGraph::Node prevNode=solver.predNode(v);
      DiGraph::Arc arc= lemon::findArc(complexCaseGraph,prevNode,v,lemon::INVALID);

      if(this->isMoving[arc]){
    	  //TODO delete after test
    	  rrt_planning::Cell endCell=nodeCell[this->complexToNormal[v]];
    	  rrt_planning::Cell startCell=nodeCell[this->complexToNormal[solver.predNode(v)]];
    	  this->planner.makePlan(startCell,endCell,vec);
    	  reverse(vec.begin(),vec.end());
    	  result.insert(result.end(),vec.begin(),vec.end());
    	  vec.clear();

      }
      }
    std::cout<<"beg id"<<std::endl;
    for (DiGraph::Node v=endCompl;v != firstCompl; v=solver.predNode(v)) {
        std::cout<<"node ids"<<this->complexCaseGraph.id(v)<<std::endl;
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

void planner::ComplexPlanner::createComplexGraph(rrt_planning::Cell* start,rrt_planning::Cell* end){
	createComplexNodes(start,end);
	std::cout<<"creation"<<std::endl;
	createArcsSameNodes();
	std::cout<<"same node connection"<<std::endl;
	connectDifferentNodes();
	std::cout<<"different node connection"<<std::endl;
	this->connectFirstCell(start);
	std::cout<<"first cell"<<std::endl;
		this->lastCellConnection(end);
		std::cout<<"last cell"<<std::endl;
}

void planner::ComplexPlanner::connectFirstCell(rrt_planning::Cell* start){
	DiGraph::Node firstNode=this->cellNode[*start];
	DiGraph::Node firstComplexNode=this->nodeToVec[firstNode].at(0);
	//for loop to connect with the other nodes where the first node is not a transmition node
	for(DiGraph::OutArcIt out(graph,firstNode);out!=INVALID;++out){
		//next node in the normal graph connected
		DiGraph::Node nod=graph.target(out);
		int sp;
		if(grid->isComm(this->nodeCell[nod],&sp)){
		std::vector<rrt_planning::Cell> path=this->arcPath[out];
		int distance=this->length[out];
		std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
		//TODO find when discretization parameter is not exact
		DiGraph::Arc addedArc=this->complexCaseGraph.addArc(firstComplexNode,vecEndNodes.at(distance/this->discretizationPar));
		this->lengthComplex[addedArc]=distance;
		this->arcPath[addedArc]=path;
		this->isMoving[addedArc]=true;
		}else{
			std::vector<rrt_planning::Cell> path=this->arcPath[out];
			int distance=this->length[out];
			DiGraph::Arc addedArc=this->complexCaseGraph.addArc(firstComplexNode,nodeToVec[nod].at(0));
			this->isMoving[addedArc]=true;
			this->arcPath[addedArc]=path;
			this->lengthComplex[addedArc]=distance;


		}
	}

	std::cout<<"9"<<std::endl;
}
void planner::ComplexPlanner::createComplexNodes(rrt_planning::Cell* start,rrt_planning::Cell* end){
		//first create the first and last nodes
		DiGraph::Node firstNode=this->cellNode[*start];
		DiGraph::Node firstComplexNode=complexCaseGraph.addNode();
		std::vector<DiGraph::Node> firstNodesVec;
		firstNodesVec.push_back(firstComplexNode);
		//TODO delete after testing
		for(DiGraph::Node n:firstNodesVec){
			this->complexToNormal[n]=firstNode;
		}
		this->nodeToVec[firstNode]=firstNodesVec;
		DiGraph::Node endNode=this->cellNode[*end];
		DiGraph::Node endComplexNode=complexCaseGraph.addNode();
		std::vector<DiGraph::Node> endNodesVec;
		endNodesVec.push_back(endComplexNode);
		for(DiGraph::Node n:endNodesVec){
					this->complexToNormal[n]=endNode;
				}
		this->nodeToVec[endNode]=endNodesVec;
	   /*
		 * for loop that goes through all the nodes of the graph we want to transform
		 * checks if the cell associated with the node is a communication node
		 * then transforms it
		 */
		for(DiGraph::NodeIt n(graph); n != INVALID; ++n ){
			int speed;
			std::vector<DiGraph::Node> vecNodes;
			if(this->grid->isComm(this->nodeCell[n],&speed)){
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
//TODO Tested works
void planner::ComplexPlanner::createArcsSameNodes(){
	/*
		 * for loop that creates the connections between the nodes in the
		 * complex case
		 */
		for(DiGraph::NodeIt b(graph); b != INVALID; ++b ){
				int speed;
				if(this->grid->isComm(this->nodeCell[b],&speed)){
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

void planner::ComplexPlanner::lastCellConnection(rrt_planning::Cell* end ){
	DiGraph::Node endNode=this->cellNode[*end];
	DiGraph::Node endComplexNode=this->nodeToVec[endNode].at(0);
	//add end node in the graph
		for(DiGraph::InArcIt in(graph,endNode);in!=INVALID;++in){
				//next node in the normal graph connected
				DiGraph::Node nod=graph.source(in);
				int sp;
				if(grid->isComm(this->nodeCell[nod],&sp)){

				std::vector<rrt_planning::Cell> path=this->arcPath[in];
				int distance=this->length[in];
				std::vector<DiGraph::Node> vecSourceNodes=nodeToVec[nod];
				for(int j=0;j<1+(TMAX-distance)/this->discretizationPar;j++){
					DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecSourceNodes.at(j),endComplexNode);
					this->lengthComplex[addedArc]=distance;
					this->arcPath[addedArc]=path;
					this->isMoving[addedArc]=true;
				}
				path.clear();
			}
		}
		std::cout<<"endCMP id:"<<this->complexCaseGraph.id(endComplexNode)<<std::endl;

}


void planner::ComplexPlanner::connectDifferentNodes(){
	//TODO maybe add rrumbullakimin per siper
		/*
		 * for loop that creates the connections between the nodes in the
		 * complex case
		 */
		for(DiGraph::NodeIt b(graph); b != INVALID; ++b ){
				int speed;
				//TODO works only if the start and end node are not communication cells
				if(this->grid->isComm(this->nodeCell[b],&speed)){
				std::vector<DiGraph::Node> vecStartNodes=nodeToVec[b];
				    //for loop to connect with the other nodes
					for(DiGraph::OutArcIt out(graph,b);out!=INVALID;++out){
						//next node in the normal graph connected
						DiGraph::Node nod=graph.target(out);
						int sp;
						if(!this->grid->isComm(nodeCell[nod],&sp))
							continue;
						//TODO delete after test
						try{
							arcPath.at(out);
						}catch(const std::out_of_range& oor){
							std::cout<<"error"<<std::endl;
						}
						std::vector<rrt_planning::Cell> path=this->arcPath[out];
						int distance=this->length[out];

						std::vector<DiGraph::Node> vecEndNodes=nodeToVec[nod];
						for(int j=0;j<numberOfNodes-distance/this->discretizationPar;j++){
							DiGraph::Arc addedArc=this->complexCaseGraph.addArc(vecStartNodes.at(j),vecEndNodes.at(j+distance/this->discretizationPar));
							this->lengthComplex[addedArc]=distance;
							this->arcPath[addedArc]=path;
							this->isMoving[addedArc]=true;
						}
						path.clear();
					}
				}
			}
}






} /* namespace planner */
