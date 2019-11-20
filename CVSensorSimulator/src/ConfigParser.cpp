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
			 {"cy", c.cy},
			 {"FoV_deg", c.FoV_deg}};
}

void CameraInfo::from_json(const json& j, cameraInfo& c) {
	j.at("cameraID").get_to(c.cameraID);
	j.at("x_res").get_to(c.x_res);
	j.at("y_res").get_to(c.y_res);
	j.at("fx").get_to(c.fx);
	j.at("fy").get_to(c.fy);
	j.at("cx").get_to(c.cx);
	j.at("cy").get_to(c.cy);
	j.at("FoV_deg").get_to(c.FoV_deg);
}

Config ConfigParser::getConfigs(string filepath) {
	json jsonFile;
	std::ifstream fileReader(filepath);
	if (!fileReader.is_open()) {
		cerr << "No config file found. Check config file path and/or name." << endl;
		exit(-1);
	}
	fileReader >> jsonFile;
	fileReader.close();
	Config config;
	config.visualize = jsonFile.value("visualize", true);
	config.cInfo = jsonFile["cameraInfo"].get<CameraInfo::cameraInfo>(); // @suppress("Ambiguous problem")
	config.tagsize = jsonFile.value("tagsize", 48.0);
	config.tag_plane_dist = jsonFile.value("tag_plane_dist", 1.0);
	double h = jsonFile["target_hsv_thresh_low"]["h"].get<double>();
	double s = jsonFile["target_hsv_thresh_low"]["s"].get<double>();
	double v = jsonFile["target_hsv_thresh_low"]["v"].get<double>();
	config.target_thresh_low = cv::Scalar(h,s,v);
	h = jsonFile["target_hsv_thresh_high"]["h"].get<double>();
	s = jsonFile["target_hsv_thresh_high"]["s"].get<double>();
	v = jsonFile["target_hsv_thresh_high"]["v"].get<double>();
	config.target_thresh_high = cv::Scalar(h,s,v);
	for (uint i = 0; i < jsonFile["Robots"].size(); i++) {
		int tagID = jsonFile["Robots"][i]["tagID"].get<int>(); // @suppress("Ambiguous problem")
		std::string label = jsonFile["Robots"][i]["label"].get<std::string>(); // @suppress("Ambiguous problem")
		shared_ptr<Robot> robot = make_shared<Robot>(tagID, label, config.cInfo.x_res, config.cInfo.y_res, config.tag_plane_dist);
		config.robots.push_back(robot);
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
