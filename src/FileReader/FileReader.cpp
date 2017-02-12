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
	std::fstream myfile("projectFiles/mapFiles/blocked_map/map.txt");
	int i,j;
	for(i=0;i<100;i++){
		for(j=0;j<100;j++){
			myfile>> (*mat)(i,j);
		}
	}
	myfile.close();

}


