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
#include "ConfigParser.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

using namespace std;
using namespace cv;

bool running;
int visualize = 1;

void *vid_cap_thread(void* args) {
	FrameBuffer* fb = (FrameBuffer*)args;
	while(running) {
		fb->updateFrame();
	}
	return NULL;
}

void* detector_thread(void* args) {
	PoseDetector* pd = (PoseDetector*)args;

	Mat temp_frame(100, 100, CV_8UC3, Scalar(0,0,0));
	Mat* frame = &temp_frame;
	imshow("RobotDetections", *frame);

	while (running) {
		pd->updatePoseEstimates();
		if(visualize) {
			frame = pd->getLabelledFrame();
			imshow("RobotDetections", *frame);
		}
		// ESC key to exit
		if (waitKey(30) == 27) {
			running = false;
		}
	}
	return NULL;
}

int main(int argc, char* argv[]) {
	running = true;

	ConfigParser cp;
	Config config;
	if (argc > 1) {
		config = cp.getConfigs(argv[1]);
	}
	else {
		cerr << "No config file path provided." << endl;
	}

	apriltag_detection_info_t info;

	info.tagsize = config.tagsize;
	info.fx = config.fx;
	info.fy = config.fy;
	info.cx = config.cx;
	info.cy = config.cy;

	VidCapSettings settings;

	settings.cameraID = config.cameraID;
	settings.x_res = config.x_res;
	settings.y_res = config.y_res;

	int size = config.robots.size();
	vector<TaggedObject> robots(size);
	int i = 0;
	while (config.robots.size() > 0) {
		robots[i] = config.robots.front();
		config.robots.pop_front();
		i++;
	}

	FrameBuffer fb(settings);

	PoseDetector pd(&fb, info, &robots, size);

	pthread_t threads [2];

	int rc = pthread_create(&threads[0], NULL, vid_cap_thread, &fb);
	if (rc) {
		cout << "Error:unable to create video capture thread," << rc << endl;
		exit(-1);
	}

	rc = pthread_create(&threads[1], NULL, detector_thread, &pd);
	if (rc) {
		cout << "Error:unable to create detector thread," << rc << endl;
		exit(-1);
	}

	//TODO replace with socket/server code
	while (running) {
		pose2D pose = robots[0].getPose();
		cout << "x:" << pose.x << " y:" << pose.y << " yaw:" << pose.yaw << endl;
	}

	void* status;
	for (int i = 0; i < 2; i++) {
		pthread_join (threads[i], &status);
	}

	destroyAllWindows();
	return 0;
}
