
#include <map/CommMap.h>
#include "xml/xmlParser.h"
#include <iostream>
#include <SDL2/SDL.h>
#include "view/View.h"
#include "Planner.h"
#include "FileReader/FileReader.h"
#include "map/DebugMap.h"
#include "grid/Grid.h"
#include "ComplexPlanner.h"
#include "planners/ThetaStarPlanner.h"
#include "view/OpenCvView.h"

void printMat(MatrixDyn* mat){

	for(int i=0;i<mat->rows();i++){
		std::cout<<std::endl;
		for(int j=0;j<mat->cols();j++){
			std::cout<<(*mat)(i,j)<<" ";
		}
	}
}


int main(int argc, char* argv[])
{

    char *filePath="projectFiles/lemon_graph/thetaGraph";
    char *filePathGr="projectFiles/lemon_graph/gridGraph";
    int baseUnit=8;
    	int baseRate=2;
    	const int buff= 20;
	//create communication map which has the information about the antennas and the speed of transmittion in
    //different cells
    std::cout<<"here"<<std::endl;
    MatrixDyn mat(75,100);
	CommMap grid(&mat);
	std::cout<<"starting"<<std::endl;
	xml::xmlParser parse;
	parse.parse();
	grid.setMatrix(parse.getAntenne(),baseRate);
	//create blocked matrix which has the information about blocked and free cells
	MatrixDyn bl(75,100);
	FileReader r;
	std::cout<<"std"<<std::endl;
	r.reader(&bl);
	//create debug map which the actual map that the solver uses to get the information about cells
	DebugMap map(&bl,&grid);
	//create grid
	Grid gridMap(map,75,100);

	planner::Planner pl(&gridMap);
	//create the two different planners that are going to be used to calculate the distance and the path between
	//two communication nodes
	//ThetaStarPlanner planner(&gridMap);
	planner::GridPlanner plan(&gridMap);
	plan.createGraph();
	//create complex planner one that uses theta* and the other that uses four way grid
	//planner::ComplexPlanner compPl(&gridMap,disPar,&planner,filePath);

	planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,filePathGr,buff);

	//compPl.createGraphs();
	grPlanner.createGraphs();

	srand(time(NULL));
	//int x, y,xend,yend;
	std::vector<Cell> c;
	/*
	for(int i=0;i<10000;i++){

			x = rand()%100;
			y = rand()%100;
			xend=rand()%100;
			yend=rand()%100;
			if(gridMap.isFree(std::make_pair(x,y))&&gridMap.isFree(std::make_pair(xend,yend))){
				std::cout<<"("<<x<<","<<y<<")"<<"("<<xend<<","<<yend<<")"<<std::endl;
				compPl.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
				compPl.makeSimplePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
				grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
				grPlanner.makeSimplePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
			}
	}*/


	//create the view
	view::View view;
	view.setPlanner(&pl);
	//view.setComplexPlanner(&compPl);
	view.setComGridPlanner(&grPlanner);
	view.setMat(mat,bl);
	if(view.Draw()){
	    std::cout<<"success"<<std::endl;
	}else{
		std::cout<<"false"<<std::endl;
	}




}
