/*
 * ConfigParser.cpp
 *
 *  Created on: Jun. 24, 2019
 *      Author: calvin
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

	return config;
}
