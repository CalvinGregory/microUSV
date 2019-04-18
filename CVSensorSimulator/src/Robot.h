/*
 * Robot.h
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */

#ifndef ROBOT_H_
#define ROBOT_H_


#include "TaggedObject.h"

class Robot : TaggedObject {
protected:
	double boundingBox[2];
	string name;
public:
	Robot(int tagID, double size_x, double size_y, string name);
	string getName();
	double* getBoundingBox();
};


#endif /* ROBOT_H_ */
