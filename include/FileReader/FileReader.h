/*
 * FileReader.h
 *
 *  Created on: Dec 7, 2016
 *      Author: arlind
 */

#ifndef TESTFILES_FILEREADER_H_
#define TESTFILES_FILEREADER_H_
#include <map/CommMap.h>
#include <eigen3/Eigen/Dense>

class FileReader {
public:
	FileReader();
	virtual ~FileReader();
	void reader(MatrixDyn* mat);
};

#endif /* TESTFILES_FILEREADER_H_ */
