//============================================================================
// Name        : CVSensorSimulator.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <map>
#include <list>
#include <iterator>
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

enum ArgType { 	VisualizeFlag,
				CameraID,
				xResolution,
				yResolution,
				xFocalLength,
				yFocalLength,
				xCenter,
				yCenter,
				TagSize,
				Robot_,
				Puck_ };

static std::map<std::string, ArgType> argType;

void initArgType() {
	argType["visualize"] = VisualizeFlag;
	argType["cameraID"] = CameraID;
	argType["res_x"] = xResolution;
	argType["res_y"] = yResolution;
	argType["fx"] = xFocalLength;
	argType["fy"] = yFocalLength;
	argType["cx"] = xCenter;
	argType["cy"] = yCenter;
	argType["tagsize"] = TagSize;
	argType["Robot"] = Robot_;
//	argType["Puck"] = Puck_;
}

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
	initArgType();
	running = true;
	apriltag_detection_info_t info;
	VidCapSettings settings;

	list<Robot> robotList;

	if (argc > 1) {
		ifstream cFile (argv[1]);
		if (cFile.is_open()) {
			string line;
			while(getline(cFile, line)) {
				line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
				if(line[0] == '#' || line.empty()) {
					continue;
				}

				int delimiterPos = line.find("=");
				string name = line.substr(0, delimiterPos);
				string value = line.substr(delimiterPos + 1);
				switch(argType[name])
				{
					case VisualizeFlag:
						visualize = stoi(value);
						break;
					case CameraID:
						settings.cameraID = stoi(value);
						break;
					case xResolution:
						settings.x_res = stoi(value);
						break;
					case yResolution:
						settings.y_res = stoi(value);
						break;
					case xFocalLength:
						info.fx = stod(value);
						break;
					case yFocalLength:
						info.fy = stod(value);
						break;
					case xCenter:
						info.cx = stod(value);
						break;
					case yCenter:
						info.cy = stod(value);
						break;
					case TagSize:
						info.tagsize = stod(value);
						break;
					case Robot_:
						int delim1 = value.find(",");
						string id = value.substr(0, delim1);
						string label = value.substr(delim1 + 1);
						Robot robot(stoi(id), 0, 0, label);
						robotList.push_back(robot);
						break;
//					case Puck_:
//						//TODO add Puck parser
//						break;
				}
			}
		}
		else {
			cerr << "Could not open config file " << argv[1] << endl;
		}

		cFile.close();
	}
	int size = robotList.size();
	vector<TaggedObject> robots(size);
	int i = 0;
	while (robotList.size() > 0) {
		robots[i] = robotList.front();
		robotList.pop_front();
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
//		cout << "x:" << pose.x << " y:" << pose.y << " yaw:" << pose.yaw << endl;
	}

	void* status;
	for (int i = 0; i < 2; i++) {
		pthread_join (threads[i], &status);
	}

	destroyAllWindows();
	return 0;
}
