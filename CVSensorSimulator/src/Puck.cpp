/*
 * Puck.cpp
 *
 *  Created on: Apr. 19, 2019
 *      Author: calvin
 */

#include "Puck.h"

using namespace std;

Puck::Puck(int tagID, double radius, Color color) {
	this->tagID = tagID;
	this->label = "puck";
	this->radius = radius;
	this->color = color;
	sem_init(&mutex, 0, 1);
	tagRGB = make_tuple(0, 255, 0);
}

Puck::~Puck() {
	sem_destroy(&mutex);
}

double Puck::getRadius() {
	return radius;
}
