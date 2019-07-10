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

#include "ConfigParser.h"

using namespace ConfigParser;
using namespace CameraInfo;
using namespace std;

void CameraInfo::to_json(json& j, const cameraInfo& c) {
	j = json{{"cameraID", c.cameraID},
			 {"x_res", c.x_res},
			 {"y_res", c.y_res},
			 {"fx", c.fx},
			 {"fy", c.fy},
			 {"cx", c.cx},
			 {"cy", c.cy}};
}

void CameraInfo::from_json(const json& j, cameraInfo& c) {
	j.at("cameraID").get_to(c.cameraID);
	j.at("x_res").get_to(c.x_res);
	j.at("y_res").get_to(c.y_res);
	j.at("fx").get_to(c.fx);
	j.at("fy").get_to(c.fy);
	j.at("cx").get_to(c.cx);
	j.at("cy").get_to(c.cy);
}

Config ConfigParser::getConfigs(string filepath) {
	json jsonFile;
	std::ifstream fileReader(filepath);
	fileReader >> jsonFile;
	fileReader.close();
	Config config;
	config.visualize = jsonFile.value("visualize", true);
	config.cInfo = jsonFile["cameraInfo"].get<CameraInfo::cameraInfo>(); // @suppress("Ambiguous problem")
	config.tagsize = jsonFile.value("tagsize", 48.0);
	for (uint i = 0; i < jsonFile["Robots"].size(); i++) {
		int tagID = jsonFile["Robots"][i]["tagID"].get<int>(); // @suppress("Ambiguous problem")
		std::string label = jsonFile["Robots"][i]["label"].get<std::string>(); // @suppress("Ambiguous problem")
		Robot robot(tagID, 0, 0, label);
		config.robots.push_back(robot);
	}
	for (uint i = 0; i < jsonFile["Pucks"].size(); i++) {
		int tagID = jsonFile["Pucks"][i]["tagID"].get<int>(); // @suppress("Ambiguous problem")
		Puck puck(tagID, 25, BLUE);
		config.pucks.push_back(puck);
	}
	for (uint i = 0; i < jsonFile["Obstacles"].size(); i++) {
		int tagID = jsonFile["Obstacles"][i]["tagID"].get<int>(); // @suppress("Ambiguous problem")
		Obstacle obstacle(tagID);
		config.obstacles.push_back(obstacle);
	}
	for (uint i = 0; i < jsonFile["Waypoints"].size(); i++) {
		Waypoint waypoint;
		waypoint.x = jsonFile["Waypoints"][i]["x"].get<int>(); // @suppress("Ambiguous problem")
		waypoint.y = jsonFile["Waypoints"][i]["y"].get<int>(); // @suppress("Ambiguous problem")
		config.waypoints.push_back(waypoint);
	}
	config.loop_waypoints = jsonFile.value("loop_waypoints", false);
	config.output_csv = jsonFile.value("output_csv", false);
	return config;
}
