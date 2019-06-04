/*
 * Robot.cpp
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */


#include "Robot.h"

using namespace std;

// TODO add sensor zone positions/sizes

Robot::Robot(int tagID, double size_x, double size_y, string label) {
	this->tagID = tagID;
	this->label = label;
	boundingBox[0] = size_x;
	boundingBox[1] = size_y;
	sem_init(&mutex, 0, 1);
	tagRGB = make_tuple(0, 0, 255);
}

Robot::~Robot() {
	sem_destroy(&mutex);
}

double* Robot::getBoundingBox() {
	return boundingBox;
}
