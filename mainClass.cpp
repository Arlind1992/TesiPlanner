
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
#include "planners/BaseLinePlanner.h"
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
	std::ofstream myfile;
    myfile.open("RandomBaseRate8");
    int baseUnit=8;
    	int baseRate=2;
    	const int buffer= 10;
	//create communication map which has the information about the antennas and the speed of transmittion in
    //different cells
    std::cout<<"here"<<std::endl;
    MatrixDyn mat(75,100);
	CommMap grid(&mat);
	std::cout<<"mod"<<std::endl;
	xml::xmlParser parse;
	parse.parse();
	grid.setMatrix(parse.getAntenne(),baseRate);
	//create blocked matrix which has the information about blocked and free cells
	MatrixDyn bl(75,100);
	FileReader r;
	r.reader(&bl);
	//create debug map which the actual map that the solver uses to get the information about cells
	DebugMap map(&bl,&grid);
	//create grid
	Grid gridMap(map,75,100);

	planner::Planner pl(&gridMap);
	planner::BaseLinePlanner basePlan(&gridMap,baseUnit);
	basePlan.createGraph();

	planner::GridPlanner plan(&gridMap);
	plan.createGraph();
	//create complex planner one that uses theta* and the other that uses four way grid
//planner::ComplexPlanner compPl(&gridMap,disPar,&planner,filePath);

	srand(time(NULL));
	int x, y,xend,yend;
	std::vector<Cell> c;
	std::map<Cell,int> buffV;
	int counter=0;
	planner::Baseline baseL(&gridMap,baseUnit,baseRate,buffer,&myfile,&basePlan);
			baseL.createGraph();

			planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,buffer,&myfile);

			//compPl.createGraphs();
			grPlanner.createGraphs();


	//Test random Points
			//Second Set
	for(int buff=10;buff<=60;buff=buff+10){
		counter=0;
		myfile<<"Buffer Size "<<buff<<std::endl;
	planner::Baseline baseLTest(&gridMap,baseUnit,baseRate,buff,&myfile,&basePlan);
		baseLTest.createGraph();

		planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,buff,&myfile);

		//compPl.createGraphs();
		grPlanner.createGraphs();

	while(counter<100){

			x = rand()%75;
			y = rand()%100;
			xend=rand()%75;
			yend=rand()%100;
			if(gridMap.isFree(std::make_pair(x,y))&&gridMap.isFree(std::make_pair(xend,yend))){
				std::cout<<"("<<x<<","<<y<<")"<<"("<<xend<<","<<yend<<")"<<std::endl;

				if(!grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c,buffV)){
					myfile<<"No Solution Complex Case"<<std::endl;
				}
				if(!baseLTest.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c)){
					myfile<<"No Solution Baseline"<<std::endl;
				}
				counter++;
			}
	}
	}
	//Test With base Rate 2
	std::ofstream myfile2;
	    myfile2.open("RandomBaseRate2");
	int baseRate2=2;
    MatrixDyn mat2(75,100);
	CommMap grid2(&mat2);
	xml::xmlParser parse2;
	parse2.parse();
	grid2.setMatrix(parse2.getAntenne(),baseRate2);
	//create debug map which the actual map that the solver uses to get the information about cells
	DebugMap map2(&bl,&grid2);
	//create grid
	Grid gridMap2(map2,75,100);
	planner::GridPlanner plan2(&gridMap2);
			plan2.createGraph();

	planner::BaseLinePlanner basePlan2(&gridMap2,baseUnit);
	basePlan2.createGraph();

	for(int buff=10;buff<=60;buff=buff+10){
		std::cout<<buff<<std::endl;
		counter=0;
		myfile2<<"Buffer Size "<<buff<<std::endl;
	planner::Baseline baseLTest(&gridMap2,baseUnit,baseRate2,buff,&myfile2,&basePlan2);
		baseLTest.createGraph();

		planner::ComplexPlanner grPlanner(&gridMap2,baseUnit,baseRate2,&plan2,buff,&myfile2);

		//compPl.createGraphs();
		grPlanner.createGraphs();

	while(counter<100){

			x = rand()%75;
			y = rand()%100;
			xend=rand()%75;
			yend=rand()%100;
			if(gridMap.isFree(std::make_pair(x,y))&&gridMap.isFree(std::make_pair(xend,yend))){
				std::cout<<"("<<x<<","<<y<<")"<<"("<<xend<<","<<yend<<")"<<std::endl;

				if(!grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c,buffV)){
					myfile2<<"No Solution Complex Case"<<std::endl;
				}
				if(!baseLTest.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c)){
					myfile2<<"No Solution Baseline"<<std::endl;
				}
				counter++;
			}
	}
	}//END TEST Second Set

	//Test With base Rate 2
		std::ofstream myfile3;
		    myfile3.open("FixedBaseRate8");
	//FIRST SET TEST
	for(int bufferTest1=1;bufferTest1<=60;bufferTest1++){
		myfile3<<"Buffer "<<bufferTest1<<std::endl;
		planner::Baseline baseLTest(&gridMap,baseUnit,baseRate,bufferTest1,&myfile3,&basePlan);
				baseLTest.createGraph();

				planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,bufferTest1,&myfile3);

				//compPl.createGraphs();
				grPlanner.createGraphs();
				std::vector<Cell> vNew;

				if(!baseLTest.makePlan(std::make_pair(27,96),std::make_pair(68,40),vNew))
					myfile3<<"No Solution baseline"<<std::endl;
				if(!grPlanner.makePlan(std::make_pair(27,96),std::make_pair(68,40),c,buffV))
					myfile3<<"No Solution Complex Case"<<std::endl;
	}
	std::ofstream myfile4;
	    myfile4.open("FixedBaseRate2");

	for(int bufferTest2=1;bufferTest2<61;bufferTest2++){
		myfile4<<"Buffer "<<bufferTest2<<std::endl;
	planner::Baseline baseLTest(&gridMap2,baseUnit,baseRate2,bufferTest2,&myfile4,&basePlan2);
		baseLTest.createGraph();

		planner::ComplexPlanner grPlanner(&gridMap2,baseUnit,baseRate2,&plan2,bufferTest2,&myfile4);

		grPlanner.createGraphs();
		if(!baseLTest.makePlan(std::make_pair(27,96),std::make_pair(68,40),c))
			myfile4<<"No solution baseline"<<std::endl;
					if(!grPlanner.makePlan(std::make_pair(27,96),std::make_pair(68,40),c,buffV))
						myfile4<<"No solution Complex Case"<<std::endl;
	}










	//create the view
	view::View view;
	view.setPlanner(&pl);
	view.setBaselinePlanner(&baseL);
	view.setBlPlan(&basePlan);
	//view.setComplexPlanner(&compPl);
	view.setComGridPlanner(&grPlanner);
	view.setMat(mat,bl);
	if(view.Draw()){
	    std::cout<<"success"<<std::endl;
	}else{
		std::cout<<"false"<<std::endl;
	}




}
