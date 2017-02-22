/*
 * View.cpp
 *
 *  Created on: Jan 4, 2017
 *      Author: arlind
 */

#include <map/CommMap.h>
#include "view/View.h"
#include <SDL2/SDL.h>
#include<iostream>
#include "Planner.h"

namespace view {

View::View() {
	width=800;
	height=600;
	this->numCellH=200;
	this->numCellV=150;
	this->endMessageShown=false;
	this->startMessageShown=false;
	this->startSelected=false;
	this->endSelected=false;
	this->solution=false;
	this->complexSolution=false;
	this->grCompSolution=false;
	this->grNSolution=false;


}

View::~View() {
	// TODO Auto-generated destructor stub
}
bool View::Draw(){
		    SDL_Event event;
		    SDL_Window* window = NULL;
		    SDL_Surface *screen = NULL;
		    int keypress = 0;

		    if (SDL_Init(SDL_INIT_VIDEO) < 0 ) return false;

		    SDL_Init(SDL_INIT_VIDEO);              // Initialize SDL2

		        // Create an application window with the following settings:
		        window = SDL_CreateWindow(
		            "An SDL2 window",                  // window title
		            SDL_WINDOWPOS_UNDEFINED,           // initial x position
		            SDL_WINDOWPOS_UNDEFINED,           // initial y position
		            WIDTH,                               // width, in pixels
		            HEIGHT,                               // height, in pixels
		            SDL_WINDOW_OPENGL                  // flags - see below
		        );

		        // Check that the window was successfully created
		        if (window == NULL) {
		            // In the case that the window could not be made...
		            printf("Could not create window: %s\n", SDL_GetError());
		            return 1;
		        }

		        // The window is open: could enter program loop here (see SDL_PollEvent())

		        //SDL_Delay(3000);  // Pause execution for 3000 milliseconds, for example

		        screen=SDL_GetWindowSurface(window);
		       while(!keypress)
			    {



			         DrawScreen(screen);
			         SDL_UpdateWindowSurface( window );
			         if(this->startSelected){
			        	 if(this->blocked(start.first,start.second)==1){
			        		 SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,"","Blocked cell select another",window);
			        					        				   	      this->startSelected=false;
			         				}
			         }
			         if(!this->startMessageShown){
			        	 SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,"","Select the cell where to start the algorithm",window);
			        	 startMessageShown=true;
			         }else{
			        	 if(this->startSelected && !this->endMessageShown){
			        		 SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,"","Select the cell where to end the algorithm",window);
			        		 endMessageShown=true;
			        	 }
			         }


			         while(SDL_PollEvent(&event))
			         {
			              switch (event.type)
			              {
			                  case SDL_QUIT:
				              keypress = 1;
				              break;
			                  case SDL_KEYDOWN:
			                       keypress = 1;
			                       break;
			                  case SDL_MOUSEBUTTONDOWN:
			                	  this->handleInput(event);
			                	  if(this->endSelected&&!this->solution){
			                		  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION,"","Solution Not Found",window);
			                		  this->endMessageShown=false;
			                		  					this->startMessageShown=false;
			                		  					this->startSelected=false;
			                		  					this->endSelected=false;
			                		  					this->solution=false;
			                		  					this->vecSolution.clear();
			                	  }
			                	  break;
			              }
			         }

}


		        // Close and destroy the window
		        SDL_DestroyWindow(window);

		        // Clean up
		        SDL_Quit();


		    return true;
}
void View::drawMat(SDL_Surface *screen)
	{
	    SDL_FillRect(screen,nullptr,SDL_MapRGB(screen->format, 255, 255, 255));
	    int pixelsForMatX=4;
	    int pixelsForMatY=4;
	    int rectX=0,rectY=0;
	    for(int i=0;i<150;i++){
	    	for(int j=0;j<200;j++){
	    		SDL_Rect rect;
	    		rect.x=rectX;
	    		rect.y=rectY;
	    		rect.h=pixelsForMatY;
	    		rect.w=pixelsForMatX;
	    		int mat=toDraw(i,j);
	    		int blocked=this->blocked(i,j);
	    		SDL_FillRect(screen,&rect,this->selectColor(screen,mat,blocked));
	    		if(this->startSelected){
	    			if(start.first==i&&start.second==j){
	    				SDL_FillRect(screen,&rect,SDL_MapRGB(screen->format,0,0,255));
	    			}
	    		}
	    		rectX=rectX+pixelsForMatX;

	    	}
	    	rectX=0;
	    	rectY=rectY+pixelsForMatY;
	    }

	}
void View::DrawScreen(SDL_Surface* screen)
	{

	    if(SDL_MUSTLOCK(screen))
	    {
	        if(SDL_LockSurface(screen) < 0) return;
	    }

	    drawMat(screen);
/*
	    if(this->solution){
	    	this->drawSolution(screen,this->vecSolution,1);
	    }
	    if(this->complexSolution)
	    	this->drawSolution(screen,this->vecSolutionComplex,2);
	    */
	    if(this->grNSolution)
	    	this->drawSolution(screen,this->vecGrSolution,3);

	    if(this->grCompSolution)
	    	this->drawSolution(screen,this->vecGrComplexSolution,4);

	    if(SDL_MUSTLOCK(screen)) SDL_UnlockSurface(screen);


	}
void View::setMat(MatrixDyn mat,MatrixDyn block){
	this->toDraw=mat;
	this->blocked=block;
}


int View::selectColor(SDL_Surface* screen,int power,int blocked){
	if(blocked==1){
		return SDL_MapRGB(screen->format,0,0,0);
	}
	if(blocked==2){
		return SDL_MapRGB(screen->format,128,128,128);
	}
	if(power==0){
		return SDL_MapRGB(screen->format, 255, 255, 255);
	}
	return SDL_MapRGB(screen->format, 255, 200-10*power, 200-10*power);
}
Cell View::pixelToMatCoo(int x,int y){
	int xMat=x/(this->width/this->numCellH);
	int yMat=y/(this->height/this->numCellV);
	return std::make_pair(yMat,xMat);
}
void View::drawSolution(SDL_Surface *screen,std::vector<rrt_planning::Cell> vec,int complex){

	for(int i=0;i<vec.size()-1;i++){
		Cell cOne=matToPixelCoo(vec[i]);
		Cell cTwo=matToPixelCoo(vec[i+1]);
		drawLine(screen,cOne.first,cOne.second,cTwo.first,cTwo.second,complex);
	}

}
Cell View::matToPixelCoo(Cell cell){
	//TODO change when making a movable camera
	int x=cell.first*this->height/this->numCellV+(this->height/this->numCellV)/2;
	int y=cell.second*this->width/this->numCellH+(this->width/this->numCellH)/2;
	return std::make_pair(x,y);
}
void View::setSol(bool so){
	this->solution=so;
}
void View::setVecSol(std::vector<Cell> vec){
	this->vecSolution=vec;
}
void View::drawLine(SDL_Surface* screen,int y1,int x1,int y2,int x2,int complex){
	 // Bresenham's line algorithm
	SDL_Rect r;
	r.h=3;
	r.w=3;
	    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
	    if(steep)
	    {
	        std::swap(x1, y1);
	        std::swap(x2, y2);
	    }

	    if(x1 > x2)
	    {
	        std::swap(x1, x2);
	        std::swap(y1, y2);
	    }

	    const float dx = x2 - x1;
	    const float dy = fabs(y2 - y1);

	    float error = dx / 2.0f;
	    const int ystep = (y1 < y2) ? 1 : -1;
	    int y = (int)y1;

	    const int maxX = (int)x2;

	    for(int x=(int)x1; x<maxX; x++)
	    {
	        if(steep)
	        {
	        	r.x=y;
	        	r.y=x-1;
	        	switch(complex){
	        	case 1:
	        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0, 255, 0));
	        		break;
	        	case 2:
	        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0,0, 255));
	        		break;
	        	case 3:
	        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 255,255, 0));
	        		break;
	        	case 4:
	        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 127,0, 255));
	        		break;
	        	}
	        }
	        else

	        {
	        	r.x=x;
	        	r.y=y-1;
	        	switch(complex){
	        		        	case 1:
	        		        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0, 255, 0));
	        		        		break;
	        		        	case 2:
	        		        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0,0, 255));
	        		        		break;
	        		        	case 3:
	        		        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 255,255, 0));
	        		        	    break;
	        		        	case 4:
	        		        		SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 127,0, 255));
	        		        		break;
	        		        	}
	        }

	        error -= dy;
	        if(error < 0)
	        {
	            y += ystep;
	            error += dx;
	        }
	    }

}


void View::handleInput(SDL_Event event){
	int x=event.button.x;
		int y=event.button.y;
		if(event.button.button==SDL_BUTTON_LEFT){
			if(!this->startSelected){

				this->start=this->pixelToMatCoo(x,y);
				startSelected=true;
			}else{
				if(!this->endSelected){
					this->end=this->pixelToMatCoo(x,y);
					endSelected=true;
					//find a way to select the Tmax
					//this->solution=this->complexPlan->makeSimplePlan(end,start,this->vecSolution);
					//this->complexSolution=this->complexPlan->makePlan(end,start,this->vecSolutionComplex);
					this->grNSolution=this->grPlanner->makeSimplePlan(end,start,this->vecGrSolution);
					this->grCompSolution=this->grPlanner->makePlan(end,start,this->vecGrComplexSolution);
					this->solution=this->grNSolution;
					this->complexSolution=this->grCompSolution;
				}
			}

		}else{
			if(event.button.button==SDL_BUTTON_RIGHT){
				if(startSelected&&endSelected){
					this->endMessageShown=false;
					this->startMessageShown=false;
					this->startSelected=false;
					this->endSelected=false;
					this->solution=false;
					this->complexSolution=false;
					this->grCompSolution=false;
					this->grNSolution=false;
					this->vecGrComplexSolution.clear();
					this->vecGrSolution.clear();
					this->vecSolutionComplex.clear();
					this->vecSolution.clear();
				}


			}
		}
}

void View::setPlanner(planner::Planner* plan){
	this->plan=plan;
}
void View::setComplexPlanner(planner::ComplexPlanner* cmpPlaner){
	this->complexPlan=cmpPlaner;
}

void View::setComGridPlanner(planner::ComplexPlanner* grPlanner){
	this->grPlanner=grPlanner;
}
} /* namespace view */
