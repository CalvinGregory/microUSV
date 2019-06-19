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

#ifndef CVSS_UTIL_H_
#define CVSS_UTIL_H_


#include <vector>
#include "TaggedObject.h"

namespace CVSS_util {
	/*
	 * Searches the list of TaggedObjects for one who's tagID matches the target tagID. If found,
	 * return the index of the TaggedObject inside the objects vector.
	 *
	 * @param objects The TaggedObject vector to search.
	 * @param tagID The tagID number to search for.
	 *
	 * @return If an object with a matching tagID is found, return the object's index, else return -1.
	 */
	int tagMatch(std::vector<TaggedObject>* objects, int tagID);
}


#endif /* CVSS_UTIL_H_ */
