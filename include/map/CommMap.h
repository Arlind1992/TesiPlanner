/*
 * CommGrid.h
 *
 *  Created on: Dec 29, 2016
 *      Author: arlind
 */

#ifndef COMMGRID_H_
#define COMMGRID_H_
#include <eigen3/Eigen/Dense>
#include "xml/xmlParser.h"
using namespace Eigen;
using namespace xml;
typedef Matrix<int,Eigen::Dynamic,Eigen::Dynamic> MatrixDyn;
class CommMap {
public:
	CommMap(MatrixDyn* mat):repMatrix(mat){}
	virtual ~CommMap();
	void setMatrix(std::vector<xml::antenna> antenne,bool discrete);
	bool getSpeed(int x,int y,int* speed);

private:
	MatrixDyn* repMatrix;
	void setSpeed(int xCenter,int yCenter,int Speed,bool discrete);
};

#endif /* COMMGRID_H_ */
