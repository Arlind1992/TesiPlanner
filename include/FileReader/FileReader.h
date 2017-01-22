/*
 * FileReader.h
 *
 *  Created on: Dec 7, 2016
 *      Author: arlind
 */

#ifndef TESTFILES_FILEREADER_H_
#define TESTFILES_FILEREADER_H_
#include <eigen3/Eigen/Dense>
#include "grid/CommGrid.h"

class FileReader {
public:
	FileReader();
	virtual ~FileReader();
	void reader(MatrixDyn* mat);
};

#endif /* TESTFILES_FILEREADER_H_ */
