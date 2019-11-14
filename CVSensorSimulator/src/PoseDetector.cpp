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

#include "PoseDetector.h"
#include "Robot.h"
using namespace cv;
using namespace std;

PoseDetector::PoseDetector(apriltag_detection_info_t detInfo, vector<shared_ptr<Robot>> robots) {
	this->detInfo = detInfo;
	this->robots = robots;
	detections = NULL;
	tf = tag25h9_create();
	td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
}

PoseDetector::~PoseDetector() {
	apriltag_detector_destroy(td);
	tag25h9_destroy(tf);
	zarray_destroy(detections);
}

void PoseDetector::updatePoseEstimates(Mat* frame) {
	if (!frame->empty()) {
		this->frame = *frame;
		Mat gray;
		cvtColor(this->frame, gray, COLOR_BGR2GRAY);
		// Make an image_u8_t header for the Mat data
		image_u8_t im = { .width = gray.cols,
			.height = gray.rows,
			.stride = gray.cols,
			.buf = gray.data
		};

		detections = apriltag_detector_detect(td, &im);
		apriltag_detection_t* det;
		for(int i = 0; i < zarray_size(detections); i++) {
			zarray_get(detections, i, &det);
			int index = CVSS_util::tagMatch(robots, det->id);
			if (index < 0) { // Remove all detections which do not belong to a valid TaggedObject
				zarray_remove_index(detections, i, 0);
				i--;
			}
			else { // Estimate the pose of the TaggedObject
				apriltag_pose_t pose;
				detInfo.det = det;
				estimate_tag_pose(&detInfo, &pose);
				robots.at(index)->setPose(pose3Dto2D(pose, AngleUnit::RADIANS));
			}
		}
	}
}

Mat* PoseDetector::getLabelledFrame(ConfigParser::Config config) {
	apriltag_detection_t* det;
	// Label each TaggedObject in the image.
	for(int i = 0; i < zarray_size(detections); i++) {
		zarray_get(detections, i, &det);
		label_tag_detection(&frame, det);
	}

	// Draw waypoints on frame
	uint r = 255;
	uint g = 0;
	uint b = 0;
	int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
	int waypoint_number = 0;
	for (std::vector<ConfigParser::Waypoint>::iterator it = config.waypoints.begin(); it != config.waypoints.end(); it++) {
		circle(frame, Point(it->x + config.cInfo.cx, it->y + config.cInfo.cy), 2, Scalar(b, g, r), 2);
		// circle(frame, Point(it->x + config.cInfo.cx, it->y + config.cInfo.cy), 50, Scalar(b, g, r), 2);
		double fontscale = 1.0;
		cv::String text = cv::String(to_string(waypoint_number));
		putText(frame, text, Point(it->x + 15 + config.cInfo.cx, it->y + config.cInfo.cy), fontface, fontscale, Scalar(b, g, r), 2);
		waypoint_number++;
	}

	return &frame;
}

pose2D PoseDetector::pose3Dto2D(apriltag_pose_t pose, AngleUnit unit) {
	pose2D pose2D;
	gettimeofday(&pose2D.timestamp, NULL);
	pose2D.yaw = atan2(pose.R->data[3], pose.R->data[0]); //radians
	if (unit == AngleUnit::DEGREES)
		pose2D.yaw = pose2D.yaw * 180 / 3.14159;
	pose2D.x = pose.t->data[0];
	pose2D.y = pose.t->data[1];
	return pose2D;
}

void PoseDetector::label_tag_detection(Mat* frame, apriltag_detection_t* det) {
	int index = CVSS_util::tagMatch(robots, det->id);
	int fontface;
	String text;
	if (!(index < 0)) {
		text = robots.at(index)->getLabel();
		fontface = FONT_HERSHEY_DUPLEX;
	}
	else {
		std::stringstream ss;
		ss << det->id;
		text = ss.str();
		fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
	}

	// Outline the AprilTag in the TaggedObject's color.
	uint r = get<0>(robots.at(index)->getTagColor());
	uint g = get<1>(robots.at(index)->getTagColor());
	uint b = get<2>(robots.at(index)->getTagColor());
	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(b, g, r), 2);

	// Place the TaggedObject's label at the apriltag's location.
	double fontscale = 1.0;
	int baseline;
	Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
	putText(*frame, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, Scalar(0, 0x8c, 0xf0), 2);
}
