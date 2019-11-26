/**
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

/**
 * SensorZone is a struct containing the dimensions to define a triangle representing a sensor's field of view. 
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

/**
 * SensorValues is a struct containing all data contained in a SensorData message. 
 * 
 * pose The Robot's current 2D pose estimate.
 * targetSensors List of target sensor values. Sensors are defined in order starting from the port side and moving clockwise around the vessel.
 * obstacleSensors List of obstacle sensor values. Sensors are defined in order starting from the port side and moving clockwise around the vessel.
 * cluster_point_range Distance from the Robot to the cluster point (i.e. center pixel of the camera frame)
 * cluster_point_heading Heading angle from the Robot's current position to the cluster point (i.e. center pixel of the camera frame)
 */ 
typedef struct {
	pose2D pose;
	std::vector<bool> targetSensors;
	std::vector<bool> obstacleSensors;
	double cluster_point_range;
	double cluster_point_heading;
} SensorValues;

/**
 * The Robot class represents a microUSV marked with an AprilTag. Each instance stores the current state of the microUSV.
 */
class Robot : public TaggedObject {
protected:
	int x_res;
	int y_res;
	double boundingBox[2];
	std::mutex sensorVal_lock;
	SensorValues sensorVals_complete;
	SensorValues sensorVals_incomplete;
	std::vector<SensorZone> sensors;
	/**
	 * Function builds a vector of sensor mask images, one for each sensor, with the same dimensions as a camera frame. 
	 * Each sensor mask is black with a white triangle. The triangle's dimensions and position in the mask
	 * are defined by the sensorZone dimensions and the provided pose.
	 * 
	 * @param pose The pose of the Robot from which to draw the sensor zones.
	 * @param px_per_mm Conversion factor from pixels to mm in the camera frame. 
	 * 
	 * @return Vector of images (masks), one for each target sensor defined in this robot. 
	 */
	std::vector<cv::Mat> getSensorMasks(pose2D pose, double px_per_mm);
	double getTargetRange(pose2D my_pose, pose2D target_pose);
	double getTargetHeading(pose2D my_pose, pose2D target_pose);
public:
	/**
	 * @param tagID This Robot's tagID.
	 * @param label This Robot's label string.
	 * @param img_width Camera frame x dimension in pixels.
	 * @param img_height Camera frame y dimension in pixels
	 */
	Robot(int tagID, std::string label, int img_width, int img_height);
	~Robot();
	/**
	 * @return The Robot's bounding box dimensions stored as a an array of length 2 {x, y}.
	 */
	double* getBoundingBox();
	/**
	 * Updates the SensorValues stored in this Robot based on the provided list of Robot poses and target mask.
	 * 
	 * @param targets An image mask of all targets detected in the camera frame. Pixels of the target color
	 * are white while all other pixels are black.
	 * @param robot_poses Vector of poses for each Robot in the system.
	 * @param my_index The index of this robot in the robot_poses vector.
	 * @param px_per_mm Conversion factor from pixels to mm in the camera frame. 
	 */
	void updateSensorValues(cv::Mat targets, std::vector<pose2D> robot_poses, int my_index, double px_per_mm); 
	/**
	 * Retrieves the most recent SensorValues stored in this Robot. This function is thread safe.
	 * 
	 * @return A SensorValues struct populated with the most up to date data. 
	 */
	SensorValues getSensorValues();
};


#endif /* ROBOT_H_ */
