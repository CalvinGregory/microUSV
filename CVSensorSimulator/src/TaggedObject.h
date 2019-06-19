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

#ifndef TAGGEDOBJECT_H_
#define TAGGEDOBJECT_H_


#include <iostream>
#include <tuple>
#include <sys/time.h>
#include "semaphore.h"

/*
 *
 */
typedef struct {
	double x = 0;
	double y = 0;
	double yaw = 0;
	struct timeval timestamp;
} pose2D;

/*
 *
 */
class TaggedObject {
protected:
	int tagID;
	pose2D pose;
	std::string label;
	sem_t mutex;
	std::tuple<uint, uint, uint> tagRGB;
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
	std::tuple<uint, uint, uint> getTagColor() { return tagRGB; }
};


#endif /* TAGGEDOBJECT_H_ */
