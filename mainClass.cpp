
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

void printMat(MatrixDyn* mat){

	for(int i=0;i<mat->rows();i++){
		std::cout<<std::endl;
		for(int j=0;j<mat->cols();j++){
			std::cout<<(*mat)(i,j)<<" ";
		}
	}
}

void testFixed8Base(){
	std::ofstream myfile;
		std::vector<Cell> c;
		srand(time(NULL));
			std::ofstream myfile3;
			    myfile3.open("FixedBaseRate8onlybaseline");
			    int baseUnit=8;
			    	int baseRate=8;
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
				planner::GridPlanner plan(&gridMap);
				planner::BaseLinePlanner basePlan(&gridMap,baseUnit,&plan);
				basePlan.createGraph();


				//planner::GridPlanner plan(&gridMap);
				//plan.createGraph();
				for(int bufferTest1=1;bufferTest1<=55;bufferTest1++){
						myfile3<<"Buffer "<<bufferTest1<<std::endl;
						planner::Baseline baseLTest(&gridMap,baseUnit,baseRate,bufferTest1,&myfile3,&basePlan,&plan);
								baseLTest.createGraph();

								//planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,bufferTest1,&myfile3);

								//compPl.createGraphs();
								//grPlanner.createGraphs();
								std::vector<Cell> vNew;

								if(!baseLTest.makePlan(std::make_pair(27,96),std::make_pair(68,40),vNew))
									myfile3<<"No Solution baseline"<<std::endl;
								//if(!grPlanner.makePlan(std::make_pair(27,96),std::make_pair(68,40),c,buffV))
									//myfile3<<"No Solution Complex Case"<<std::endl;
					}

}




void testFixed2Base(){
	std::ofstream myfile;
myfile.open("FBaseRate2onlybaseline");
int baseUnit=8;
	int baseRate=2;
	//const int buffer= 15;
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

planner::GridPlanner plan(&gridMap);

planner::Planner pl(&gridMap);
planner::BaseLinePlanner basePlan(&gridMap,baseUnit,&plan);
basePlan.createGraph();

//create complex planner one that uses theta* and the other that uses four way grid
//planner::ComplexPlanner compPl(&gridMap,disPar,&planner,filePath);

srand(time(NULL));
int x, y,xend,yend;
std::vector<Cell> sd;
std::vector<Cell> c;
std::map<Cell,int> buffV;
int counter=0;
	for(int buffer=1;buffer<51;buffer++){
		myfile<<"Buffer "<<buffer<<std::endl;
		planner::Baseline baseL(&gridMap,baseUnit,baseRate,buffer,&myfile,&basePlan,&plan);
		baseL.createGraph();

		planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,buffer,&myfile);

		//compPl.createGraphs();
		//grPlanner.createGraphs();

		if(!baseL.makePlan(std::make_pair(27,96),std::make_pair(68,40),sd))
			myfile<<"No Solution baseline"<<std::endl;
		/*if(!grPlanner.makePlan(std::make_pair(27,96),std::make_pair(68,40),c,buffV))
			myfile<<"No Solution Complex Case"<<std::endl;*/
	}
}





void testRandom8Base(){
	std::ofstream myfile;
	std::vector<Cell> c;
	srand(time(NULL));
		int x, y,xend,yend;
		std::map<Cell,int> buffV;
		int counter=0;
		    myfile.open("RandomBaseRate8onlybaseline");
		    int baseUnit=8;
		    	int baseRate=8;
			//create communication map which has the information about the antennas and the speed of transmittion in
		    //different cells
		    MatrixDyn mat(75,100);
			CommMap grid(&mat);
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

			planner::GridPlanner plan(&gridMap);
			planner::BaseLinePlanner basePlan(&gridMap,baseUnit,&plan);
			basePlan.createGraph();


	//Test random Points
			//Second Set

	for(int buff=10;buff<=50;buff=buff+10){
		counter=0;
		myfile<<"Buffer Size "<<buff<<std::endl;
		std::cout<<"Buffer Size "<<buff<<std::endl;
	planner::Baseline baseLTest(&gridMap,baseUnit,baseRate,buff,&myfile,&basePlan,&plan);
		baseLTest.createGraph();

		//planner::ComplexPlanner grPlanner(&gridMap,baseUnit,baseRate,&plan,buff,&myfile);

		//compPl.createGraphs();
		//grPlanner.createGraphs();
		int istanza=1;
			while(counter<100){


			x = rand()%75;
			y = rand()%100;
			xend=rand()%75;
			yend=rand()%100;
			if(gridMap.isFree(std::make_pair(x,y))&&gridMap.isFree(std::make_pair(xend,yend))){
				myfile<<"Istanza :"<<istanza<<std::endl;

				/*if(!grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c,buffV)){
					myfile<<"No Solution Complex Case"<<std::endl;
				}*/
				if(!baseLTest.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c)){
					myfile<<"No Solution Baseline"<<std::endl;
				}
				counter++;
				istanza++;
				std::cout<<"END"<<std::endl;
			}

	}
	}
}



void testRandom2Base(){
	std::ofstream myfile2;
	    myfile2.open("RandomBaseRate2");
	  int baseUnit=8;
		int baseRate2=2;
		MatrixDyn bl(75,100);
			FileReader r;
			r.reader(&bl);
		    MatrixDyn mat2(75,100);
			CommMap grid2(&mat2);
			std::vector<Cell> c;
			std::map<Cell,int> buffV;
			xml::xmlParser parse2;
			parse2.parse();
			grid2.setMatrix(parse2.getAntenne(),baseRate2);
			//create debug map which the actual map that the solver uses to get the information about cells
			DebugMap map2(&bl,&grid2);
			//create grid
			Grid gridMap2(map2,75,100);
			planner::GridPlanner plan2(&gridMap2);

			planner::BaseLinePlanner basePlan2(&gridMap2,baseUnit,&plan2);
			basePlan2.createGraph();



		int counter;
		int x,y,xend,yend;

		for(int buff=10;buff<=50;buff=buff+10){
			std::cout<<buff<<std::endl;
			counter=0;
			myfile2<<"Buffer Size "<<buff<<std::endl;
		planner::Baseline baseLTest(&gridMap2,baseUnit,baseRate2,buff,&myfile2,&basePlan2,&plan2);
			baseLTest.createGraph();

			planner::ComplexPlanner grPlanner(&gridMap2,baseUnit,baseRate2,&plan2,buff,&myfile2);

			//compPl.createGraphs();
			//grPlanner.createGraphs();
			int istanza=1;
		while(counter<101){

				x = rand()%75;
				y = rand()%100;
				xend=rand()%75;
				yend=rand()%100;
				if(gridMap2.isFree(std::make_pair(x,y))&&gridMap2.isFree(std::make_pair(xend,yend))){
					 myfile2<<"Istanza :"<<istanza<<std::endl;

					/*if(!grPlanner.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c,buffV)){
						myfile2<<"No Solution Complex Case"<<std::endl;
					}*/
					if(!baseLTest.makePlan(std::make_pair(xend,yend),std::make_pair(x,y),c)){
						myfile2<<"No Solution Baseline"<<std::endl;
					}
					counter++;
					istanza++;
				}
		}
		}
}




int main(int argc, char* argv[])
{
	//testFixed8Base();
	testFixed2Base();
	//testRandom2Base();
	//testRandom8Base();

/*
	std::ofstream myfile;
    myfile.open("RandomBaseRate8");
    int baseUnit;
    	int baseRate;
    	int buffer;
    	int chosenPlan;
	//create communication map which has the information about the antennas and the speed of transmittion in
    //different cells
    MatrixDyn mat(75,100);
	CommMap grid(&mat);
	xml::xmlParser parse;
	parse.parse();
	//create blocked matrix which has the information about blocked and free cells
	MatrixDyn bl(75,100);
	FileReader r;
	r.reader(&bl);
	r.loadConfig(baseRate,baseUnit,buffer,chosenPlan);
	grid.setMatrix(parse.getAntenne(),baseRate);

	//create debug map which the actual map that the solver uses to get the information about cells
	DebugMap map(&bl,&grid);
	//create grid
	Grid gridMap(map,75,100);

	planner::GridPlanner plan(&gridMap);

	planner::BaseLinePlanner basePlan(&gridMap,baseUnit,&plan);
	basePlan.createGraph();

	planner::BaseLinePlanner basePlan3(&gridMap,baseUnit,&plan);
	basePlan3.createGraph();

	planner::BaseLinePlanner basePlan2(&gridMap,baseUnit,&plan);
	basePlan2.createGraph();

	//planner::ComplexPlanner compPl(&gridMap,baseUnit,baseRate,&plan,buffer,&myfile);
    	//	compPl.createGraphs();21,29,41;17,26,40

	planner::Baseline baseL(&gridMap,baseUnit,baseRate,29,&myfile,&basePlan,&plan);
			baseL.createGraph();
	planner::Baseline baseL2(&gridMap,baseUnit,baseRate,21,&myfile,&basePlan3,&plan);
			baseL2.createGraph();
	planner::Baseline baseL3(&gridMap,baseUnit,baseRate,41,&myfile,&basePlan2,&plan);
			baseL3.createGraph();

	//create the view
	view::View view;
	view.setBaselinePlanner(&baseL);

	view.setBaselinePlanner2(&baseL2);
	view.setBaselinePlanner3(&baseL3);
	//view.setComGridPlanner(&compPl);
	view.setMat(mat,bl);
	//view.pl=&plan;
	if(view.Draw()){
	    std::cout<<"success"<<std::endl;
	}else{
		std::cout<<"false"<<std::endl;
	}



*/
}
