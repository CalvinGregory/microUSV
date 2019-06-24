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

#ifndef CONFIGPARSER_H_
#define CONFIGPARSER_H_


#include <iostream>
#include <list>
#include <fstream>
#include "json.hpp"
#include "Robot.h"
#include "Puck.h"
#include "Obstacle.h"

using json = nlohmann::json;

namespace CameraInfo {
	/*
	 * Struct containing all camera related settings to be extracted from config file.
	 */
	typedef struct{
		int cameraID;
		int x_res;
		int y_res;
		double fx;
		double fy;
		double cx;
		double cy;
	} cameraInfo;

	/*
	 * Function called by nlohmann::json to convert a cameraInfo object to a json object.
	 *
	 * @param j The destination json object.
	 * @param c The cameraInfo object to convert.
	 */
	void to_json(json& j, const cameraInfo& c);

	/*
	 * Function called by nlohmann::json to convert a json object to a cameraInfo object.
	 *
	 * @param j The json object to convert.
	 * @param c The destination cameraInfo object.
	 */
	void from_json(const json& j, cameraInfo& c);
}

namespace ConfigParser {
	/*
	 * Struct defining cartesian coordinates of a waypoint.
	 */
	typedef struct {
		float x;
		float y;
	} Waypoint;

	/*
	 * Struct containing all data extracted from a json config file.
	 */
	typedef struct{
		bool visualize;
		CameraInfo::cameraInfo cInfo;
		double tagsize;
		std::list<Robot> robots;
		std::list<Puck> pucks;
		std::list<Obstacle> obstacles;
		std::list<Waypoint> waypoints;
		bool loop_waypoints;
	} Config;

	/*
	 * Extracts configuration data from a json config file.
	 *
	 * @param filepath The path to the json config file to be parsed.
	 */
	Config getConfigs(std::string filepath);
}


#endif /* CONFIGPARSER_H_ */
