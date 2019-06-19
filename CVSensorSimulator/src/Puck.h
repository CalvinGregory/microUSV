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

#ifndef PUCK_H_
#define PUCK_H_


#include "TaggedObject.h"

/*
 * Enum representing the possible colors of puck.
 */
enum Color{	RED,
			GREEN,
			BLUE
};

/*
 * The Puck class represents a floating puck marked with an AprilTag. It stores the current state of the puck.
 */
class Puck : public TaggedObject {
protected:
	Color color;
	double radius;
public:
	/*
	 * @param tagID This Puck's tagID number.
	 * @param radius The radius of this Puck in mm.
	 * @param color This Puck's color.
	 */
	Puck(int tagID, double radius, Color color);
	~Puck();
	/*
	 * @return This puck's radius in mm.
	 */
	double getRadius();
};


#endif /* PUCK_H_ */
