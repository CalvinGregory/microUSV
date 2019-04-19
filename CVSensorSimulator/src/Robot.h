/*
 * Robot.h
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */

#ifndef ROBOT_H_
#define ROBOT_H_


#include "TaggedObject.h"

class Robot : public TaggedObject {
protected:
	double boundingBox[2];
public:
	Robot(int tagID, double size_x, double size_y, std::string label);
	~Robot();
	double* getBoundingBox();
};


#endif /* ROBOT_H_ */
