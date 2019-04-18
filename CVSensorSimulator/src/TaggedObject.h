/*
 * TaggedObject.h
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */

#ifndef TAGGEDOBJECT_H_
#define TAGGEDOBJECT_H_


#include <iostream>

typedef struct{
	double x;
	double y;
	double yaw;
} pose2D;

class TaggedObject {
protected:
	int tagID;
	pose2D pose;
	std::string label;
public:
	int getTagID() { return tagID; }
	void setPose(pose2D pose) { this->pose = pose; }
	pose2D getPose() { return pose; }
	std::string getLabel() { return label; }
};


#endif /* TAGGEDOBJECT_H_ */
