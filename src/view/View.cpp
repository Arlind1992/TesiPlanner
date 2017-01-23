/*
 * View.cpp
 *
 *  Created on: Jan 4, 2017
 *      Author: arlind
 */

#include "view/View.h"
#include <SDL2/SDL.h>
#include<iostream>
#include "grid/CommGrid.h"
#include "Planner.h"

namespace view {

View::View() {
	width=WIDTH;
	height=HEIGHT;
	this->numCellH=100;
	this->numCellV=100;
	this->camCenterX=numCellH/2;
	this->camCenterY=numCellV/2;
	this->endMessageShown=false;
	this->startMessageShown=false;
	this->startSelected=false;
	this->endSelected=false;
	this->solution=false;


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
			       std::cout<<"vdsklsda"<<std::endl;


		        // Close and destroy the window
		        SDL_DestroyWindow(window);

		        // Clean up
		        SDL_Quit();


		    return true;
}
void View::drawMat(SDL_Surface *screen, int xCamCenter, int yCamCenter)
	{
	    SDL_FillRect(screen,nullptr,SDL_MapRGB(screen->format, 255, 255, 255));
	    int maxX=toDraw.rows();
	    int maxY=toDraw.cols();
	    int pixelsForMatX;
	    int pixelsForMatY;
	    pixelsForMatX=this->width/this->numCellH;
	    pixelsForMatY=this->height/this->numCellV;
	    int rectX=0,rectY=0;
	    int begi,endi,begj,endj;
	    if(xCamCenter<this->numCellH/2){
	    	begi=0;
	    	endi=this->numCellH;
	    }else{
	    	if(xCamCenter+this->numCellH/2<maxX){
	    		begi=maxX-this->numCellH;
	    		endi=maxX;
	    	}else{
	    		begi=xCamCenter-this->numCellH/2;
	    		endi=xCamCenter+this->numCellH/2;
	    	}

	    }
	    if(yCamCenter<this->numCellV/2){
	    	begj=0;
	    	endj=this->numCellV;
	    }else{
	    	if(yCamCenter+this->numCellV/2<maxY){
	    		begj=maxY-this->numCellV;
	    		endj=maxY;
	    	}else{
	    		begj=yCamCenter-this->numCellV/2;
	    		endj=yCamCenter-this->numCellV/2;
	    	}
	    }

	    for(int i=begi;i<endi;i++){
	    	for(int j=begj;j<endj;j++){
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

	    drawMat(screen,10,10);
	    if(this->solution){
	    	this->drawSolution(screen);
	    }

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
	if(power==0){
		return SDL_MapRGB(screen->format, 255, 255, 255);
	}
	return SDL_MapRGB(screen->format, 255, 100-10*power, 100-10*power);
}
Cell View::pixelToMatCoo(int x,int y){
	int xMat=x/(this->width/this->numCellH);
	int yMat=y/(this->height/this->numCellV);
	return std::make_pair(yMat,xMat);
}
void View::drawSolution(SDL_Surface *screen){

	for(int i=0;i<vecSolution.size()-1;i++){
		Cell cOne=matToPixelCoo(vecSolution[i]);
		Cell cTwo=matToPixelCoo(vecSolution[i+1]);
	/*std::cout<<"pixel "<<cOne.first<<std::endl;
		std::cout<<"pixel "<<cOne.second<<std::endl;
		std::cout<<"pixel "<<cTwo.first<<std::endl;
		std::cout<<"pixel "<<cTwo.second<<std::endl;

		//drawLine(screen,76,547,148,619);
*/
		drawLine(screen,cOne.first,cOne.second,cTwo.first,cTwo.second);

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
void View::drawLine(SDL_Surface* screen,int y1,int x1,int y2,int x2){
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
	        	r.y=x;
	            SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0, 255, 0));
	        }
	        else

	        {
	        	r.x=x;
	        	r.y=y;
	            SDL_FillRect(screen,&r, SDL_MapRGB(screen->format, 0, 255, 0));
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
				std::cout<<start.first<<std::endl;
				std::cout<<start.second<<std::endl;
				startSelected=true;
			}else{
				if(!this->endSelected){
					this->end=this->pixelToMatCoo(x,y);
					std::cout<<end.first<<std::endl;
									std::cout<<end.second<<std::endl;
					endSelected=true;
					//find a way to select the Tmax
					std::cout<<"deri te zgjidhja"<<std::endl;
					this->solution=this->plan->makePlan(end,TMAX,start,this->vecSolution);
					planner::Planner::stampVector(vecSolution);
					std::cout<<"mbas zgjidhjes"<<std::endl;


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
					this->vecSolution.clear();
				}


			}
		}
}

void View::setPlanner(planner::Planner* plan){
	this->plan=plan;
}


} /* namespace view */
