/*
 * CVSensorSimulator tracks the pose of objects fitted with AprilTags in view of
 * an overhead camera and sends that pose data to microUSV's over TCP.
 *
 * Copyright (C) 2019  CalvinGregory  cgregory@mun.ca
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
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
	gettimeofday(&this->pose.timestamp, NULL);
}

Puck::~Puck() {
	sem_destroy(&mutex);
}

double Puck::getRadius() {
	return radius;
}
