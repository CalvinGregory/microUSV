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

#ifndef POSEDETECTOR_H_
#define POSEDETECTOR_H_


#include "FrameBuffer.h"
#include "TaggedObject.h"
#include <math.h>
#include <sys/time.h>
#include "CVSS_util.h"
#include "ConfigParser.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

/*
 * Enum representing the possible angle units.
 */
enum class AngleUnit {
	RADIANS,
	DEGREES
};

/*
 * Wrapper class of the AprilTag pose detection algorithm from https://github.com/AprilRobotics/apriltag.
 *
 * PoseDetector object detects AprilTags in a provided image. It removes all tags which were not specified
 * as belonging to a tagged object when the program was initialized. It then estimates the two dimensional
 * pose of each remaining tag and stores it in the corresponding TaggedObject.
 */
class PoseDetector {
private:
	FrameBuffer* fb;
	apriltag_detection_info_t detInfo;
	std::vector<TaggedObject>* objects;

	cv::Mat frame;
	apriltag_family_t* tf;
	apriltag_detector_t* td;
	zarray_t* detections;

	/*
	 * Converts 3D pose data to 2D pose data.
	 *
	 * @param pose 3D pose data from the apriltag detector algorithm.
	 * @param unit Specifies if output should be in Radians or Degrees.
	 *
	 * @return The AprilTag's pose in 2D space.
	 */
	pose2D pose3Dto2D(apriltag_pose_t pose, AngleUnit unit);

	/*
	 * Adds an outline and label string to the frame at the provided apriltag's location.
	 *
	 * @param frame Pointer to the frame to modify.
	 * @param det Pointer to the object containing the detection info for a tag detected in the provided frame.
	 */
	void label_tag_detection(cv::Mat* frame, apriltag_detection_t* det);
public:
	/*
	 * @param fb Pointer to the FrameBuffer object which captures video frames.
	 * @param detInfo Detection info required by the apriltag detection algorithm.
	 * @param objects Pointer to the global list of valid TaggedObjects.
	 */
	PoseDetector(FrameBuffer* fb, apriltag_detection_info_t detInfo, std::vector<TaggedObject>* objects);

	~PoseDetector();

	/*
	 * Pulls a new frame from the frame buffer and estimates the poses of all TaggedObjects detected in the frame.
	 */
	void updatePoseEstimates();

	/*
	 * Adds outlines and labels to all TaggedObjects detected in the frame.
	 *
	 * @param config Config file data from ConfigParser. Contains list of waypoints and camera info.
	 *
	 * @return The frame with added tag outlines and labels.
	 */
	cv::Mat* getLabelledFrame(ConfigParser::Config config);
};


#endif /* POSEDETECTOR_H_ */
