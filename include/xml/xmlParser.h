/*
 * xmlParser.h
 *
 *  Created on: Dec 29, 2016
 *      Author: arlind
 */

#ifndef XMLPARSER_H_
#define XMLPARSER_H_
#include <tinyxml.h>
#include <string>
#include <vector>
#include "grid/Cell.h"
#include "tinyxml2.h"
using namespace rrt_planning;
namespace xml{
typedef std::pair<rrt_planning::Cell,int> antenna;
class xmlParser {
public:
	static const char* FILEPATH;
	xmlParser();
	virtual ~xmlParser();
	void parse( );
	std::vector<antenna> getAntenne();

private:
	std::vector<antenna> antenne;

};
}
#endif /* XMLPARSER_H_ */
