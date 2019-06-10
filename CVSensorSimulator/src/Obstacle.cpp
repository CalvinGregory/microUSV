/*
 * Obstacle.cpp
 *
 *  Created on: Jun. 4, 2019
 *      Author: calvin
 */

#include "Obstacle.h"

using namespace std;

//Stub class

Obstacle::Obstacle(int tagID) {
	this->tagID = tagID;
	this->label = "obstacle";
	sem_init(&mutex, 0, 1);
	tagRGB = make_tuple(255, 0, 0);
	gettimeofday(&this->pose.timestamp, NULL);
}

Obstacle::~Obstacle() {
	sem_destroy(&mutex);
}
