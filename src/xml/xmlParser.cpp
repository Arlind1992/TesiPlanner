/*
 * xmlParser.cpp
 *
 *  Created on: Dec 29, 2016
 *      Author: arlind
 */

#include "xml/xmlParser.h"
#include <string>
#include "xml/tinyxml2.h"

using namespace xml;

xmlParser::xmlParser() {


}

xmlParser::~xmlParser() {
	// TODO Auto-generated destructor stub
}
const char* xmlParser::FILEPATH="mapFiles/antenne/antenna.xml";


void xmlParser::parse(  )
{
	tinyxml2::XMLDocument doc;
	if(doc.LoadFile(FILEPATH)!=0){
		std::cout<<"Can't load file"<<std::endl;
		return;
	}

	tinyxml2::XMLElement* pChild;
	tinyxml2::XMLElement* parent=doc.FirstChildElement();
	for ( pChild = parent->FirstChildElement(); pChild != 0; pChild = pChild->NextSiblingElement())
		{
		tinyxml2::XMLElement *data;
		int i=0;
		int x=0,y=0,speed=0;
		for(data=pChild->FirstChildElement();data!=0;data=data->NextSiblingElement()){

			switch (i){
			case 0:
				x=data->DoubleText();
				break;
			case 1:
				y=data->DoubleText();
				break;
			case 2:
				speed=data->DoubleText();
				break;
			}
			i++;
			if(i==3){
				antenne.push_back(std::make_pair(std::make_pair(x,y),speed));
			}

		}

		}

}


std::vector<xml::antenna> xml::xmlParser::getAntenne(){
	return this->antenne;
}

