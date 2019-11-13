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

Robot::Robot(int tagID, double size_x, double size_y, string label, vector<sensorZone> sensors, int img_width, int img_height, double tag_plane_dist) {
	this->tagID = tagID;
	this->label = label;
	boundingBox[0] = size_x;
	boundingBox[1] = size_y;
	this->sensors = sensors;
	this->x_res = img_width;
	this->y_res = img_height;
	this->tag_plane_dist = tag_plane_dist;
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

Mat Robot::getSensorMask(int idx, pose2D pose) {
	Mat mask(y_res, x_res, CV_8U, Scalar(0,0,0));

	// cv::floodFill()?

	//DEBUG
	imshow("sensor_mask", mask);
	waitKey(0);

	return mask;
}

vector<cv::Mat> Robot::getSensorMasks(int img_width, int img_height, double tag_plane_dist) {
	vector<cv::Mat> masks;
	for (int i = 0; i < sensors.size(); i++) {
		masks.push_back(getSensorMask(i, this->pose));
	}
	return masks;
}
