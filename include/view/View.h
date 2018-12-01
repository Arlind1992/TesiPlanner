/*
 * View.h
 *
 *  Created on: Jan 4, 2017
 *      Author: arlind
 */

#ifndef VIEW_H_
#define VIEW_H_
#include <map/CommMap.h>
#include <planners/GridPlanner.h>
#include <SDL2/SDL.h>
#include "Planner.h"
#include "ComplexPlanner.h"
#include "Baseline.h"
#define WIDTH 800
#define HEIGHT 600
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
	void setComplexPlanner2(planner::ComplexPlanner* cmpPlaner);
	void setComplexPlanner3(planner::ComplexPlanner* cmpPlaner);

	void setComGridPlanner(planner::ComplexPlanner* grPlanner);
	void setGridPlanner(planner::GridPlanner *pl);
	void setBaselinePlanner(planner::Baseline *base);
	void setBaselinePlanner2(planner::Baseline *base);
	void setBaselinePlanner3(planner::Baseline *base);
	rrt_planning::ThetaStarPlanner* pl;
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
	std::vector<Cell> vecGrComplexSolution;
	std::vector<Cell> baselineSolution;
	std::vector<Cell> baselineSolution2;
	std::vector<Cell> baselineSolution3;
	bool solution;
	bool complexSolution;
	bool grCompSolution;
	bool baselineSol;
	bool baselineSol2;
	bool baselineSol3;
	bool blSolutionVar;


	//the functions that actually draw
	void DrawScreen(SDL_Surface* screen);
	void drawMat(SDL_Surface *screen);
	void drawSolution(SDL_Surface *screen,std::vector<Cell> vec,int complex);
	void DrawScreenSameBaseLine(SDL_Surface* screen);

	//Helper functions
	//TODO function to be modified depending on the speed transmition for the
	//moment maximum transmition speed 10
	int selectColor(SDL_Surface* screen,int power,int blocked);

    void drawLine(SDL_Surface *screen,int x1,int y1,int x2,int y2,int complex);
    Cell matToPixelCoo(Cell cell);
    Cell pixelToMatCoo(int x,int y);

    //Funtion to handle the communication with the user
    void handleInput(SDL_Event event);

    //Planner
    planner::ComplexPlanner* grPlanner;
    planner::ComplexPlanner* grPlanner1;

    planner::Baseline *baseline;
    planner::Baseline *baseline2;
    planner::Baseline *baseline3;
    void drawComplexSolution(SDL_Surface *screen,std::vector<Cell> vec,std::vector<int> bufferSt);
    void addNumberForCell(SDL_Surface *screen,Cell s,int i);
    void handleInputMultipleBaseline(SDL_Event event);



};

} /* namespace view */

#endif /* VIEW_H_ */
