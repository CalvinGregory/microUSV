/*
 * Puck.cpp
 *
 *  Created on: Apr. 19, 2019
 *      Author: calvin
 */

#include "Puck.h"

using namespace std;

Puck::Puck(int tagID, double radius) {
	this->tagID = tagID;
	this->radius = radius;
	sem_init(&mutex, 0, 1);
}

Puck::~Puck() {
	sem_destroy(&mutex);
}

double Puck::getRadius() {
	return radius;
}
