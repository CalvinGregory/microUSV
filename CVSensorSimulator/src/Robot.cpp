/*
 * Robot.cpp
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */


#include "Robot.h"

Robot::Robot(int tagID, double size_x, double size_y, string name) {
	this->tagID = tagID;
	this->name = name;
	boundingBox[0] = size_x;
	boundingBox[1] = size_y;
}

string Robot::getName() {
	return name;
}

double* Robot::getBoundingBox() {
	return boundingBox;
}
