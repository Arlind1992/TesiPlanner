
#include "xml/xmlParser.h"
#include <iostream>
#include <SDL2/SDL.h>
#include "view/View.h"
#include "grid/CommGrid.h"
#include "Planner.h"
#include "FileReader/FileReader.h"
#include "map/DebugMap.h"
#include "grid/Grid.h"


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


	MatrixDyn mat(100,100);
	CommGrid grid(&mat);
	MatrixDyn bl(100,100);
	xml::xmlParser parse;
		parse.parse();
		grid.setMatrix(parse.getAntenne());
	//printMat(&mat);
		FileReader r;
		r.reader(&bl);
		DebugMap map(&bl,&grid);
	Grid gridMap(map,100,100);
	planner::Planner pl(&gridMap);
	view::View view;
	view.setPlanner(&pl);
	view.setMat(mat,bl);
	if(view.Draw()){
	    std::cout<<"success"<<std::endl;
	}else{
		std::cout<<"false"<<std::endl;
	}




}
