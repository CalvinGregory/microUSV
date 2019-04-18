/*
 * TaggedObject.h
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */

#ifndef TAGGEDOBJECT_H_
#define TAGGEDOBJECT_H_


#include <iostream>
using namespace std;

typedef struct{
	double x;
	double y;
	double yaw;
} pose2D;

class TaggedObject {
protected:
	int tagID;
	pose2D pose;
public:
	int getTagID() { return tagID; }
	void setPose(pose2D pose) { this->pose = pose; }
	pose2D getPose() { return pose; }
};


#endif /* TAGGEDOBJECT_H_ */
