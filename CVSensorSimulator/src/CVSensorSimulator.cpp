//============================================================================
// Name        : CVSensorSimulator.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "pthread.h"
#include "opencv2/opencv.hpp"
#include "FrameBuffer.h"
#include "PoseDetector.h"
#include "Robot.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

using namespace std;
using namespace cv;

bool running;
bool visualize;

void *vid_cap_thread(void* args) {
	FrameBuffer* fb = (FrameBuffer*)args;
	while(running) {
		fb->updateFrame();
	}
	return NULL;
}

void label_tag_detection(Mat* frame, apriltag_detection_t* det) {
		line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
		line(*frame, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
		line(*frame, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
		line(*frame, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

		stringstream ss;
		ss << det->id;
		String text = ss.str();
		int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontscale = 1.0;
		int baseline;
		Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
		putText(*frame, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
}

struct detector_thread_args {
	FrameBuffer* fb;
	apriltag_detection_info_t info;
};

void* detector_thread(void* args) {
	detector_thread_args* dt_args =  (struct detector_thread_args*)args;
	FrameBuffer* fb = dt_args->fb;
	apriltag_detection_info_t info = dt_args->info;

	//Initialize tag detector with options
	apriltag_family_t* tf = tag25h9_create();

	apriltag_detector_t* td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);

	Mat frame, gray;

	Mat temp_frame(100, 100, CV_8UC3, Scalar(0,0,0));
	frame = temp_frame;
	imshow("Tag Detections", frame);
	while (running) {
		frame = *fb->getFrame();
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		// Make an image_u8_t header for the Mat data
		image_u8_t im = { .width = gray.cols,
			.height = gray.rows,
			.stride = gray.cols,
			.buf = gray.data
		};

		zarray_t* detections = apriltag_detector_detect(td, &im);
		cout << zarray_size(detections) << " tags detected" << endl;
		apriltag_detection_t* det;
		apriltag_pose_t pose;
		for (int i = 0; i < zarray_size(detections); i++) {
			zarray_get(detections, i, &det);
			info.det = det;
			estimate_tag_pose(&info, &pose);

//			cout << "ID: " << det->id << endl;
//			cout << "R: ";
			for (uint j = 0; j < pose.R->ncols*pose.R->nrows; j++) {
//				cout << pose.R->data[j] << " ";
			}
//			cout << endl;
//			cout << "t: ";
			for (uint j = 0; j < pose.t->ncols*pose.t->nrows; j++) {
//				cout << pose.t->data[j] << " ";
			}
//			cout << endl << endl;
		}

		if(visualize) {
			// Draw detection outlines
			for(int i = 0; i < zarray_size(detections); i++) {
				zarray_get(detections, i, &det);
				label_tag_detection(&frame, det);
			}
			imshow("Tag Detections", frame);
		}

		if (waitKey(30) == 27) {
			cout << "detector break" << endl;
			running = false;
		}
		zarray_destroy(detections);
	}

	// cleanup
	apriltag_detector_destroy(td);
	tag25h9_destroy(tf);
	return NULL;
}

void* detector_thread_OOP(void* args) {
	PoseDetector* pd = (PoseDetector*)args;

	Mat temp_frame(100, 100, CV_8UC3, Scalar(0,0,0));
	Mat* frame = &temp_frame;
	imshow("RobotDetections", *frame);

	while (running) {
		pd->updatePoseEstimates();
		if(visualize) {
			frame = pd->getLabelledFrame();
			imshow("Robot Detections", *frame);
		}

		if (waitKey(30) == 27) {
//			cout << "detector break" << endl;
			running = false;
		}
	}
	return NULL;
}

int main() {
	running = true;
	visualize = true;
//	visualize = false;

	apriltag_detection_info_t info;
	info.tagsize = 48; //mm
	// laptop camera
//	info.fx = 1390.4361690999849;
//	info.fy = 1390.4361690999849;
//	info.cx = 640;
//	info.cy = 360;
	// C920
	info.fx = 1419.9787511737059;
	info.fy = 1419.9787511737059;
	info.cx = 960;
	info.cy = 540;

	VidCapSettings settings;
	// laptop camera
//	settings.cameraID = 0;
//	settings.x_res = 1280;
//	settings.y_res = 720;
	// C920
	settings.cameraID = 1; //FIXME if computer boots with camera plugged in it changes the camera ID of C920 to 0. I think.
	settings.x_res = 1920; //C920 campath: /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_174F315F-video-index0 -- plugged in after boot
	settings.y_res = 1080;

	FrameBuffer fb(settings);

	pthread_t threads [2];

	int rc = pthread_create(&threads[0], NULL, vid_cap_thread, &fb);
	if (rc) {
		cout << "Error:unable to create video capture thread," << rc << endl;
		exit(-1);
	}

//	detector_thread_args dt_args;
//	dt_args.fb = &fb;
//	dt_args.info = info;
//	rc = pthread_create(&threads[1], NULL, detector_thread, &dt_args);
//	if (rc) {
//		cout << "Error:unable to create detector thread," << rc << endl;
//		exit(-1);
//	}

	// detector thread attempt 2
	vector<TaggedObject> robots = { Robot(0,0,0,"zero"),
						Robot(5,0,0,"five"),
						Robot(6,0,0,"six"),
						Robot(12,0,0,"twelve") };

	PoseDetector pd(&fb, info, robots, 4);
	rc = pthread_create(&threads[1], NULL, detector_thread_OOP, &pd);
	if (rc) {
		cout << "Error:unable to create detector thread," << rc << endl;
		exit(-1);
	}

	// spin main
	//TODO replace with socket/server code
//	while (running) {}

	void* status;
	for (int i = 0; i < 2; i++) {
		pthread_join (threads[i], &status);
	}

	destroyAllWindows();
	return 0;
}

