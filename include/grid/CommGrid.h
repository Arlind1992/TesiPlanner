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
class CommGrid {
public:
	CommGrid(MatrixDyn* mat):repMatrix(mat){}
	virtual ~CommGrid();
	void setMatrix(std::vector<xml::antenna> antenne);
	bool getSpeed(int x,int y,int& speed);

private:
	MatrixDyn* repMatrix;
	void setSpeed(int xCenter,int yCenter,int Speed);
};

#endif /* COMMGRID_H_ */
