/*
 * openCvView.h
 *
 *  Created on: Feb 25, 2017
 *      Author: arlind
 */

#ifndef INCLUDE_VIEW_OPENCVVIEW_H_
#define INCLUDE_VIEW_OPENCVVIEW_H_
#include <map/CommMap.h>
#include <planners/GridPlanner.h>
#include <opencv2/opencv.hpp>
#include "Planner.h"
#include "ComplexPlanner.h"
#define WIDTH 800
#define HEIGHT 600

using namespace cv;
namespace view{
class OpenCvView{
public:
	OpenCvView(std::vector<Cell> normalSol,std::vector<Cell> complSol, std::map<Cell,int> *buf);
	bool Draw();
	void setMat(MatrixDyn toDraw,MatrixDyn blocked);


private:
	//Drawing details attribute
	int width;
	int height;
	//number of cells in horizontal
	int numCellH;
	//number of cells in vertical
	int numCellV;
	//the center of the camera calculated on the matrix
    int camCenterX;
	int camCenterY;

	//attributes to help communication with user

	Mat image;

	//attributes that set what is going to be shown in the screen that have a direct connection
		//with the planner
	MatrixDyn toDraw;
	MatrixDyn blocked;
	std::vector<Cell> vecSolution;
	std::vector<Cell> vecSolutionComplex;
	std::map<Cell,int>* stateOfBuffer;



	//the functions that actually draw
	void DrawScreen();
	void drawMat();
	void drawSolution();
	void drawCompSol();
    Cell matToPixelCoo(Cell cell);
    Cell pixelToMatCoo(int x,int y);
/*
    void drawComplexSolution(Mat *screen,std::vector<Cell> vec,std::vector<int> bufferSt);
    void addNumberForCell(Mat *screen,Cell s,int i);
  */  cv::Scalar selectColor(int power,int blocked);

    };

}




#endif /* INCLUDE_VIEW_OPENCVVIEW_H_ */
