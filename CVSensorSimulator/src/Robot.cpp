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

Robot::Robot(int tagID, double size_x, double size_y, string label, int img_width, int img_height, int x_center_px, int y_center_px, double tag_plane_dist) {
	this->tagID = tagID;
	this->label = label;
	boundingBox[0] = size_x;
	boundingBox[1] = size_y;
	this->x_res = img_width;
	this->y_res = img_height;
	this->cx = x_center_px;
	this->cy = y_center_px;
	this->tag_plane_dist = tag_plane_dist;
	sensorZone left;
	left.x_origin = -20;
	left.y_origin = 20;
	left.range = 250;
	left.heading_ang = -M_PI/4;
	left.fov_ang = M_PI/6;
	// sensorZone right;
	// right.x_origin = 20;
	// right.y_origin = 20;
	// right.range = 250;
	// right.heading_ang = M_PI/9;
	// right.fov_ang = M_PI/6;
	// sensors = {left, right};
	sensors.push_back(left);
	sem_init(&mutex, 0, 1);
	tagRGB = make_tuple(0, 0, 255);
	gettimeofday(&this->pose.timestamp, NULL);
}

Robot::~Robot() {
	sem_destroy(&mutex);
}

double* Robot::getBoundingBox() {
	return boundingBox;
}

vector<cv::Mat> Robot::getSensorMasks(double px_per_mm) {
	vector<cv::Mat> masks;
	for (int i = 0; i < sensors.size(); i++) {
		Mat mask(y_res, x_res, CV_8U, Scalar(0,0,0));

		vector<Point> FoV;
		int apex_x = cvRound(cx + this->pose.x + px_per_mm*sensors.at(i).x_origin*cos(this->pose.yaw) + px_per_mm*sensors.at(i).y_origin*sin(this->pose.yaw));
		int apex_y = cvRound(cy + this->pose.y + px_per_mm*sensors.at(i).x_origin*sin(this->pose.yaw) - px_per_mm*sensors.at(i).y_origin*cos(this->pose.yaw));
		FoV.push_back(Point(apex_x, apex_y));
		double hyp = px_per_mm*sensors.at(i).range*cos(sensors.at(i).fov_ang/2);
		int vertex_x = cvRound(apex_x + hyp*sin(this->pose.yaw + sensors.at(i).heading_ang + sensors.at(i).fov_ang/2));
		int vertex_y = cvRound(apex_y - hyp*cos(this->pose.yaw + sensors.at(i).heading_ang + sensors.at(i).fov_ang/2));
		FoV.push_back(Point(vertex_x, vertex_y));
		vertex_x = cvRound(apex_x + hyp*sin(this->pose.yaw + sensors.at(i).heading_ang - sensors.at(i).fov_ang/2));
		vertex_y = cvRound(apex_y - hyp*cos(this->pose.yaw + sensors.at(i).heading_ang - sensors.at(i).fov_ang/2));
		FoV.push_back(Point(vertex_x, vertex_y));

		vector<vector<Point>> contours {FoV};
		fillPoly(mask, contours, Scalar(255,255,255));

		masks.push_back(mask);
	}
	return masks;
}
