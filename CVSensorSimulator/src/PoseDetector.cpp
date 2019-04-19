/*
 * PoseDetector.cpp
 *
 *  Created on: Apr 15, 2019
 *      Author: calvin
 */

#include "PoseDetector.h"
using namespace cv;
using namespace std;

PoseDetector::PoseDetector(FrameBuffer* fb, apriltag_detection_info_t detInfo, vector<TaggedObject>* objects, int numObjects) {
	this->fb = fb;
	this->detInfo = detInfo;
	this->objects = objects;
	this->numObjects = numObjects;
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
		int index = tagMatch(det->id);
		if (index < 0) {
			zarray_remove_index(detections, i, 0);
			i--;
		}
		else {
			apriltag_pose_t pose;
			detInfo.det = det;
			estimate_tag_pose(&detInfo, &pose);

//			cout << "R: ";
//			for (uint j = 0; j < pose.R->ncols*pose.R->nrows; j++) {
//				cout << pose.R->data[j] << " ";
//			}
//			cout << endl;
//			cout << "t: ";
//			for (uint j = 0; j < pose.t->ncols*pose.t->nrows; j++) {
//				cout << pose.t->data[j] << " ";
//			}
//			cout << endl << endl;

//			pose2D pose2 = pose3Dto2D(pose);
//			cout << "x:" << pose2.x << " y:" << pose2.y << " yaw:" << pose2.yaw << endl;

			objects->at(index).setPose(pose3Dto2D(pose));
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

int PoseDetector::tagMatch(int tagID) {
	int index = -1;
	for (int i = 0; i < numObjects; i++) {
		if (objects->at(i).getTagID() == tagID) {
			index = i;
			break;
		}
	}
	return index;
}

pose2D PoseDetector::pose3Dto2D(apriltag_pose_t pose) {
	pose2D pose2D;
	pose2D.yaw = atan2(pose.R->data[3], pose.R->data[0]); //radians
	pose2D.yaw = pose2D.yaw * 180 / 3.14159; // DEBUG: degree conversion
	pose2D.x = pose.t->data[0];
	pose2D.y = pose.t->data[1];
	return pose2D;
}

void PoseDetector::label_tag_detection(Mat* frame, apriltag_detection_t* det) {
	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
	line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
	line(*frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
	line(*frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

	int index = tagMatch(det->id);
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

	double fontscale = 1.0;
	int baseline;
	Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
	putText(*frame, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
}
