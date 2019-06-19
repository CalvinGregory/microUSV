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

#ifndef ROBOT_H_
#define ROBOT_H_


#include "TaggedObject.h"

/*
 * The Robot class represents a microUSV marked with an AprilTag. Each instance stores the current state of the microUSV.
 */
class Robot : public TaggedObject {
protected:
	// TODO add sensor zone positions/sizes
	double boundingBox[2];
public:
	/*
	 * @param tagID This Robot's tagID.
	 * @param size_x The x dimension of this Robot's bounding box.
	 * @param size_y The y dimension of this Robot's bounding box.
	 * @param lable This Robot's label string.
	 */
	Robot(int tagID, double size_x, double size_y, std::string label);
	~Robot();
	/*
	 * @return The Robot's bounding box dimensions stored as a an array of length 2 {x, y}.
	 */
	double* getBoundingBox();
};


#endif /* ROBOT_H_ */
