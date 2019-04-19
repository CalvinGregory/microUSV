/*
 * TaggedObject.h
 *
 *  Created on: Apr. 18, 2019
 *      Author: calvin
 */

#ifndef TAGGEDOBJECT_H_
#define TAGGEDOBJECT_H_


#include <iostream>
#include "semaphore.h"

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
	sem_t mutex;
public:
	int getTagID() { return tagID; }
	void setPose(pose2D pose) {
		sem_wait(&mutex);
		this->pose = pose;
		sem_post(&mutex);
	}
	pose2D getPose() {
		pose2D temp;
		sem_wait(&mutex);
		temp = pose;
		sem_post(&mutex);
		return temp;
	}
	std::string getLabel() { return label; }
};


#endif /* TAGGEDOBJECT_H_ */
