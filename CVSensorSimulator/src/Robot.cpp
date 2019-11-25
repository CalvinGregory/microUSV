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


#include "Robot.h"

using namespace std;
using namespace cv;

Robot::Robot(int tagID, string label, int img_width, int img_height, double tag_plane_dist) {
	this->tagID = tagID;
	this->label = label;
	boundingBox[0] = 89.2;
	boundingBox[1] = 230.0;
	this->x_res = img_width;
	this->y_res = img_height;
	this->tag_plane_dist = tag_plane_dist;

	SensorZone left;
	left.x_origin = -30;
	left.y_origin = 30;
	left.range = 250;
	left.heading_ang = -M_PI/9;
	left.fov_ang = M_PI/6;
	SensorZone right;
	right.x_origin = 30;
	right.y_origin = 30;
	right.range = 250;
	right.heading_ang = M_PI/9;
	right.fov_ang = M_PI/6;
	sensors = {left, right};

	tagRGB = make_tuple(0, 0, 255);
	gettimeofday(&this->pose.timestamp, NULL);
}

Robot::~Robot() {
}

double* Robot::getBoundingBox() {
	return boundingBox;
}

vector<cv::Mat> Robot::getSensorMasks(pose2D pose, double px_per_mm) {
	vector<cv::Mat> masks;
	for (int i = 0; i < sensors.size(); i++) {
		Mat mask(y_res, x_res, CV_8U, Scalar(0,0,0));

		vector<Point> FoV;
		int apex_x = cvRound(pose.x_px + px_per_mm*(sensors.at(i).x_origin*cos(pose.yaw) + sensors.at(i).y_origin*sin(pose.yaw)));
		int apex_y = cvRound(pose.y_px + px_per_mm*(sensors.at(i).x_origin*sin(pose.yaw) - sensors.at(i).y_origin*cos(pose.yaw)));
		FoV.push_back(Point(apex_x, apex_y));
		double hyp = px_per_mm*sensors.at(i).range*cos(sensors.at(i).fov_ang/2);
		int vertex_x = cvRound(apex_x + hyp*sin(pose.yaw + sensors.at(i).heading_ang + sensors.at(i).fov_ang/2));
		int vertex_y = cvRound(apex_y - hyp*cos(pose.yaw + sensors.at(i).heading_ang + sensors.at(i).fov_ang/2));
		FoV.push_back(Point(vertex_x, vertex_y));
		vertex_x = cvRound(apex_x + hyp*sin(pose.yaw + sensors.at(i).heading_ang - sensors.at(i).fov_ang/2));
		vertex_y = cvRound(apex_y - hyp*cos(pose.yaw + sensors.at(i).heading_ang - sensors.at(i).fov_ang/2));
		FoV.push_back(Point(vertex_x, vertex_y));

		vector<vector<Point>> contours {FoV};
		fillPoly(mask, contours, Scalar(255,255,255));

		masks.push_back(mask);
	}
	return masks;
}

void Robot::updateSensorValues(Mat targets, vector<pose2D> robot_poses, int my_index, double px_per_mm) {
	sensorVals_incomplete.pose = robot_poses.at(my_index);
	sensorVals_incomplete.cluster_point_range = getTargetRange(sensorVals_incomplete.pose, pose2D());
	sensorVals_incomplete.cluster_point_heading = getTargetHeading(sensorVals_incomplete.pose, pose2D());
	
	vector<bool> targetSensors;
	vector<Mat> masks = getSensorMasks(sensorVals_incomplete.pose, px_per_mm);
	
//DEBUG
	// if (my_index == 0) {
	// 	Mat combined_masks;
	// 	bitwise_or(masks.at(0), masks.at(1), combined_masks);
	// 	char buffer [30];
	// 	int n = sprintf(buffer, "sensor_mask_%d", my_index);
	// 	imshow(buffer, combined_masks);
	// 	waitKey(1);
	// }

	for (int i = 0; i < masks.size(); i++) {
		Mat detections;
		bitwise_and(targets, masks.at(i), detections);
		targetSensors.push_back(countNonZero(detections) > 0);
	}
	sensorVals_incomplete.targetSensors = targetSensors;

	// TODO add collision sensor

	std::lock_guard<std::mutex> lock(sensorVal_lock);
	sensorVals_complete = sensorVals_incomplete;
}

SensorValues Robot::getSensorValues() {
	std::lock_guard<std::mutex> lock(sensorVal_lock);
	return sensorVals_complete;
}

double Robot::getTargetRange(pose2D my_pose, pose2D target_pose) {
	return sqrt(pow(my_pose.x - target_pose.x,2.0) + pow(my_pose.y - target_pose.y,2.0));
}

double Robot::getTargetHeading(pose2D my_pose, pose2D target_pose) {
	return atan2(target_pose.y - my_pose.y, target_pose.x - my_pose.x);
}