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
#include <vector>
#include <mutex>
#include "opencv2/opencv.hpp"

/*
 * sensorZone is a struct containing the dimensions to define a triangle representing a sensor's field of view. 
 *
 * The sensor is drawn as an isosceles triangle with its apex at coordinates (x_origin, y_origin) relative to the vessel's apriltag. 
 * The "range" value gives the triangle's height and "fov_ang" its vertex angle. The triangle's bisector line is oriented towards "heading_ang" 
 * which is measured counter clockwise relative to the vessel's heading. 
 * 
 * The coordinate system assumes the centerpoint of the apriltag is the vessel's center and the line from that center toward the vessel's bow 
 * is 0 degrees. The positive x-axis points towards the vessel's starboard side and the y-axis points towards the bow. 
 */ 
typedef struct {
	double x_origin = 0;
	double y_origin = 0;
	double range = 0;
	double heading_ang = 0;
	double fov_ang = 0;
} SensorZone;

typedef struct {
	pose2D pose;
	std::vector<bool> targetSensors;
	std::vector<bool> obstacleSensors;
	double cluster_point_range;
	double cluster_point_heading_offset;
} SensorValues;

/*
 * The Robot class represents a microUSV marked with an AprilTag. Each instance stores the current state of the microUSV.
 */
class Robot : public TaggedObject {
protected:
	int x_res;
	int y_res;
	int cx;
	int cy;
	double tag_plane_dist;
	double boundingBox[2];
	std::mutex sensorVal_lock;
	SensorValues sensorVals_complete;
	SensorValues sensorVals_incomplete;
	/*
	 *
	 */
	std::vector<cv::Mat> getSensorMasks(pose2D pose, double px_per_mm);
	double getTargetRange(pose2D my_pose, pose2D target_pose);
	double getTargetHeadingError(pose2D my_pose, pose2D target_pose);
public:
	std::vector<SensorZone> sensors;
	/*
	 * @param tagID This Robot's tagID.
	 * @param size_x The x dimension of this Robot's bounding box (mm).
	 * @param size_y The y dimension of this Robot's bounding box (mm).
	 * @param label This Robot's label string.
	 */
	Robot(int tagID, std::string label, int img_width, int img_height, int x_center_px, int y_center_px, double tag_plane_dist);
	~Robot();
	/*
	 * @return The Robot's bounding box dimensions stored as a an array of length 2 {x, y}.
	 */
	double* getBoundingBox();
	/**
	 * 
	 */
	void updateSensorValues(cv::Mat targets, std::vector<pose2D> robot_poses, int my_index, double px_per_mm); 
	/**
	 * 
	 */
	SensorValues getSensorValues();
};


#endif /* ROBOT_H_ */
