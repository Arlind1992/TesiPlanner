
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
    	int baseRate=8;
    	const int buff= 70;
    	std::cout<<"buffer "<<buff<<std::endl;
    	std::ofstream myfile;

    	myfile.open("GridPlanner");
	//create communication map which has the information about the antennas and the speed of transmittion in
    //different cells
    std::cout<<"here"<<std::endl;
    MatrixDyn mat(75,100);
	CommMap grid(&mat);
	std::cout<<"mod1"<<std::endl;
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
/*
	//TEST 1

	for(int i=1;i<61;i++){
		myfile<<"buffer "<<i<<std::endl;
		planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,filePathGr,i,&myfile);
			grPlanner.createGraphs();
		Cell s1=std::make_pair(68,39);
		Cell s2=std::make_pair(27,98);

		std::map<Cell,int> buff;
		std::vector<Cell> result;
		std::vector<Cell> resultBase;
		std::vector<Cell> simpleResult;


		if(!grPlanner.makePlan(s2,s1,result,buff))
			myfile<<"No Complex solution"<<std::endl;
		if(!grPlanner.makeSimplePlan(s2,s1,simpleResult))
			myfile<<"No Simple solution"<<std::endl;

		grPlanner.createBaseGraph();
		if(!grPlanner.makePlanBase(s2,s1,resultBase))
			myfile<<"No Baseline Solution"<<std::endl;

	}*/



	planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,filePathGr,17,&myfile);
	/*planner::ComplexPlanner grPlanner1(&gridMap,baseUnit,baseRate,&plan,filePathGr,26,&myfile);
	planner::ComplexPlanner grPlanner2(&gridMap,baseUnit,baseRate,&plan,filePathGr,25,&myfile);

	grPlanner2.createBaseGraph();
	std::cout<<"mod1"<<std::endl;
	//compPl.createGraphs();
	grPlanner.createGraphs();
	grPlanner1.createGraphs();
	grPlanner2.createGraphs();
	*/
	srand(time(NULL));
	int x, y,xend,yend;
	std::vector<Cell> c;
	std::map<Cell,int> buffV;
	for(int j=10;j<70;j=j+10){
		myfile<<"buffer "<<j<<std::endl;
		planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,filePathGr,j,&myfile);
		grPlanner.createGraphs();
		grPlanner.createBaseGraph();
		int counter=0;
		while(counter<100){
			x = rand()%75;
			y = rand()%100;
			xend=rand()%75;
			yend=rand()%100;
			if(gridMap.isFree(std::make_pair(x,y))&&gridMap.isFree(std::make_pair(xend,yend))){
				counter++;
				myfile<<"Istanza : "<<counter<<std::endl;
				std::cout<<"("<<x<<","<<y<<")"<<"("<<xend<<","<<yend<<")"<<std::endl;
			//	compPl.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
				//compPl.makeSimplePlan(std::make_pair(xend,yend),std::make_pair(x,y),c);
				if(!grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c,buffV))
					myfile<<"No complex Solution"<<std::endl;
				if(!grPlanner.makePlanBase(std::make_pair(xend,yend),std::make_pair(x,y),c))
					myfile<<"No baseline Solution"<<std::endl;
				c.clear();
			}
		}
	}



	//create the view
	view::View view;
	view.setGridPlanner(&plan);
	view.setPlanner(&pl);
	//view.setComplexPlanner(&compPl);
	//view.setComplexPlanner(&grPlanner,&grPlanner1,&grPlanner2);
	view.setMat(mat,bl);
	if(view.Draw()){
	    std::cout<<"success"<<std::endl;
	}else{
		std::cout<<"false"<<std::endl;
	}




}
