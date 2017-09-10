/*
 * FileReader.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: arlind
 */

#include "FileReader/FileReader.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
FileReader::FileReader() {
	// TODO Auto-generated constructor stub

}

FileReader::~FileReader() {
	// TODO Auto-generated destructor stub
}
void FileReader::reader(MatrixDyn* mat){
	//old map "projectFiles/mapFiles/blocked_map/map.txt"
	std::fstream myfile("map.txt");
	int i,j;
	for(i=0;i<75;i++){
		for(j=0;j<100;j++){
			myfile>> (*mat)(i,j);
		}
	}
	myfile.close();

}
void FileReader::loadConfig(int& br,int& bu,int& buffer,int& planner){
	std::fstream myfile("projectFiles/configuration/config");
    string line;
	myfile>>line;
	br=(int)line.at(line.length()-1)-(int)'0';
	myfile>>line;
	bu=(int)line.at(line.length()-1)-(int)'0';
	myfile>>line;
	buffer=(int)line.at(line.length()-1)-(int)'0'+10*((int)line.at(line.length()-2)-(int)'0');
	myfile>>line;
	planner=(int)line.at(line.length()-1)-(int)'0';

}


