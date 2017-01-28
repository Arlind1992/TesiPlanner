/*
 * View.h
 *
 *  Created on: Jan 4, 2017
 *      Author: arlind
 */

#ifndef VIEW_H_
#define VIEW_H_
#include <SDL2/SDL.h>
#include "grid/CommGrid.h"
#include "Planner.h"
#include "ComplexPlanner.h"
#define WIDTH 1000
#define HEIGHT 600
#define BPP 4
#define DEPTH 32
#define CELLSIZE 10
namespace view {

class View {
public:
	View();
	virtual ~View();
	bool Draw();
	void setMat(MatrixDyn toDraw,MatrixDyn blocked);
	void setSol(bool sol);
	void setVecSol(std::vector<Cell> vecSolution);
	void setPlanner(planner::Planner* plan);
	void setComplexPlanner(planner::ComplexPlanner* cmpPlaner);


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

	//attribute to save the starting position of the algorithm
	Cell start;
	Cell end;
	bool startSelected;
	bool startMessageShown;
	bool endSelected;
	bool endMessageShown;


	//attributes that set what is going to be shown in the screen that have a direct connection
		//with the planner
	MatrixDyn toDraw;
	MatrixDyn blocked;
	std::vector<Cell> vecSolution;
	bool solution;


	//the functions that actually draw
	void DrawScreen(SDL_Surface* screen);
	void drawMat(SDL_Surface *screen, int xCenter, int yCenter);
	void drawSolution(SDL_Surface *screen);


	//Helper functions
	//TODO function to be modified depending on the speed transmition for the
	//moment maximum transmition speed 10
	int selectColor(SDL_Surface* screen,int power,int blocked);

    void drawLine(SDL_Surface *screen,int x1,int y1,int x2,int y2);
    Cell matToPixelCoo(Cell cell);
    Cell pixelToMatCoo(int x,int y);

    //Funtion to handle the communication with the user
    void handleInput(SDL_Event event);

    //Planner
    planner::Planner* plan;
    planner::ComplexPlanner* complexPlan;



};

} /* namespace view */

#endif /* VIEW_H_ */
