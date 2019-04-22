/*
 * ConfigParser.h
 *
 *  Created on: Apr. 20, 2019
 *      Author: calvin
 */

#ifndef CONFIGPARSER_H_
#define CONFIGPARSER_H_


#include <iostream>
#include <map>
#include <list>
#include <fstream>
#include <algorithm>
#include "Robot.h"
#include "Puck.h"

typedef struct{
	int visualize;
	int cameraID;
	int x_res;
	int y_res;
	double fx;
	double fy;
	double cx;
	double cy;
	double tagsize;
	std::list<Robot> robots;
	std::list<Puck> pucks;
} Config;

class ConfigParser {
private:
	void initArgType();
public:
	ConfigParser();
	Config getConfigs(std::string filepath);
};


#endif /* CONFIGPARSER_H_ */
