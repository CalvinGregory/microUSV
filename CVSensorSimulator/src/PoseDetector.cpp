/*
 * PoseDetector.cpp
 *
 *  Created on: Apr 15, 2019
 *      Author: calvin
 */

#include "PoseDetector.h"
#include "Robot.h"
#include "Puck.h"
using namespace cv;
using namespace std;

PoseDetector::PoseDetector(FrameBuffer* fb, apriltag_detection_info_t detInfo, vector<TaggedObject>* objects) {
	this->fb = fb;
	this->detInfo = detInfo;
	this->objects = objects;
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

void PoseDetector::updatePoseEstimates() {
	frame = *fb->getFrame();
	Mat gray;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
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
		int index = CVSS_util::tagMatch(objects, det->id);
		if (index < 0) {
			zarray_remove_index(detections, i, 0);
			i--;
		}
		else {
			apriltag_pose_t pose;
			detInfo.det = det;
			estimate_tag_pose(&detInfo, &pose);
			objects->at(index).setPose(pose3Dto2D(pose, AngleUnit::RADIANS));
		}
	}
}

Mat* PoseDetector::getLabelledFrame() {
	apriltag_detection_t* det;
	for(int i = 0; i < zarray_size(detections); i++) {
		zarray_get(detections, i, &det);
		label_tag_detection(&frame, det);
	}
	return &frame;
}

pose2D PoseDetector::pose3Dto2D(apriltag_pose_t pose, AngleUnit unit) {
	pose2D pose2D;
	pose2D.yaw = atan2(pose.R->data[3], pose.R->data[0]); //radians
	if (unit == AngleUnit::DEGREES)
		pose2D.yaw = pose2D.yaw * 180 / 3.14159;
	pose2D.x = pose.t->data[0];
	pose2D.y = pose.t->data[1];
	return pose2D;
}

void PoseDetector::label_tag_detection(Mat* frame, apriltag_detection_t* det) {
	int index = CVSS_util::tagMatch(objects, det->id);
	int fontface;
	String text;
	if (!(index < 0)) {
		text = objects->at(index).getLabel();
		fontface = FONT_HERSHEY_DUPLEX;
	}
	else {
		std::stringstream ss;
		ss << det->id;
		text = ss.str();
		fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
	}

	uint r = get<0>(objects->at(index).getTagColor());
	uint g = get<1>(objects->at(index).getTagColor());
	uint b = get<2>(objects->at(index).getTagColor());

	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(b, g, r), 2);
	line(*frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(b, g, r), 2);

	double fontscale = 1.0;
	int baseline;
	Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
	putText(*frame, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, Scalar(0, 0x8c, 0xf0), 2);
}
