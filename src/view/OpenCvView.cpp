/*
 * openCvView.cpp
 *
 *  Created on: Feb 25, 2017
 *      Author: arlind
 */
#include "view/OpenCvView.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
namespace view{


OpenCvView::OpenCvView(std::vector<Cell> nSol,std::vector<Cell> cSol,std::map<Cell,int> *buffer){
	width=800;
		height=600;
		this->numCellH=100;
		this->numCellV=75;
		image = Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		this->vecSolution=nSol;
		this->vecSolutionComplex=cSol;
		this->stateOfBuffer=buffer;
}



bool OpenCvView::Draw(){

	  drawMat();
	  this->drawSolution();
	  this->drawCompSol();
	  //Create a window
	   namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	   std::cout<<"sitze= "<< this->stateOfBuffer->size()<<std::endl;
	  	       //show the image
	       imshow("My Window", image);

	  waitKey( 0 );
	  return(0);
}

void OpenCvView::drawMat(){
	int pixelsForMatX=8;
		    int pixelsForMatY=8;
		    int rectX=0,rectY=0;
		    for(int i=0;i<75;i++){
		    	for(int j=0;j<100;j++){
		    		int mat=toDraw(i,j);
		    		int blocked=this->blocked(i,j);
		    		rectangle(
		    		            image,
		    		            cvPoint(rectX,rectY),
		    		            cvPoint(rectX+pixelsForMatX,pixelsForMatY+rectY),
		    		            selectColor(mat,blocked),CV_FILLED
		    		       );


		    		rectX=rectX+pixelsForMatX;

		    	}
		    	rectX=0;
		    	rectY=rectY+pixelsForMatY;
		    }


}


cv::Scalar OpenCvView::selectColor(int power,int blocked){
	if(blocked==1){
		return cv::Scalar(0,0,0);
	}

	if(power==0){
		return cv::Scalar( 255, 255, 255);
	}
	return cv::Scalar(192-power*6, 192-power*6,255);
}
void OpenCvView::setMat(MatrixDyn toDraw,MatrixDyn blocked){
	this->toDraw=toDraw;
	this->blocked=blocked;
}
Cell OpenCvView::pixelToMatCoo(int x,int y){
	int xMat=x/(this->width/this->numCellH);
	int yMat=y/(this->height/this->numCellV);
	return std::make_pair(yMat,xMat);
}
/*
void OpenCvView::drawComplexSolution(Mat screen,std::vector<Cell> vec,std::vector<int> buffer){
	drawSolution(screen,vec,2);
	for(int i=0;i<(int)buffer.size()-1;i++){
		addNumberForCell(screen,vec.at(i),buffer.at(1));
		}
}
*/
void OpenCvView::drawSolution(){

	for(int i=0;i<this->vecSolution.size()-1;i++){
		Cell cOne=matToPixelCoo(vecSolution[i]);
		Cell cTwo=matToPixelCoo(vecSolution[i+1]);
		line(image,Point(cOne.second,cOne.first),Point(cTwo.second,cTwo.first),Scalar(0,255,0),2);
	}

}

void OpenCvView::drawCompSol(){
	int test=0;
	for(int i=0;i<this->vecSolutionComplex.size()-1;i++){
			Cell cOne=matToPixelCoo(vecSolutionComplex[i]);
			Cell cTwo=matToPixelCoo(vecSolutionComplex[i+1]);
		line(image,Point(cOne.second,cOne.first),Point(cTwo.second,cTwo.first),Scalar(255,0,0),2);
		try{
			int j = this->stateOfBuffer->at(vecSolutionComplex[i+1]);
			std::cout<<j<<std::endl;
			std::string s;
					std::stringstream out;
					out << j;
					s = out.str();
					std::cout<<"ktu-"<<s<<std::endl;
					test++;
					putText(image,s,Point(cTwo.second,cTwo.first),FONT_HERSHEY_COMPLEX_SMALL, 0.5,Scalar(0,0,0));

		}catch(const std::out_of_range& oor){

			}
			}
	std::cout<<"-"<<test<<std::endl;
}


Cell OpenCvView::matToPixelCoo(Cell cell){
	//TODO change when making a movable camera
	int x=cell.first*this->height/this->numCellV+(this->height/this->numCellV)/2;
	int y=cell.second*this->width/this->numCellH+(this->width/this->numCellH)/2;
	return std::make_pair(x,y);
}


}
