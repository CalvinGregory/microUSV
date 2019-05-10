//============================================================================
// Name        : CVSensorSimulator.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>
#include <netinet/in.h>
#include "pthread.h"
#include "opencv2/opencv.hpp"
#include <google/protobuf/util/time_util.h>

#include "FrameBuffer.h"
#include "PoseDetector.h"
#include "Robot.h"
#include "ConfigParser.h"
#include "musv_msg.pb.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

using namespace std;
using namespace cv;
using google::protobuf::util::TimeUtil;

#define PORT 8080

bool running;
int visualize = 1;

void* vid_cap_thread(void* args) {
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
		exit(-1);
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

//--- Socket --- //
	int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[128] = {0};

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	if (listen(server_fd, 10) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}

	int64 startTime = time(NULL);

	while (running) {
		mUSV::RequestData requestData;
		mUSV::SensorData sensorData;
		if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
		{
			perror("accept");
			exit(EXIT_FAILURE);
		}
		valread = read(new_socket, buffer, 128);
		if (!requestData.ParseFromString(buffer)) {
			cerr << "Failed to parse Request Data message." << endl;
			return -1;
		}

		// TODO find correct robot using requestData.tag_id()
		pose2D pose = robots[0].getPose();
//		cout << "x:" << pose.x << " y:" << pose.y << " yaw:" << pose.yaw << endl;

		sensorData.set_pose_x(pose.x);
		sensorData.set_pose_y(pose.y);
		sensorData.set_pose_yaw(pose.yaw);
		sensorData.add_obstacle_sensors(0);
		sensorData.add_obstacle_sensors(0);
		sensorData.add_obstacle_sensors(0);
		sensorData.add_puck_sensors(0);
		sensorData.add_puck_sensors(0);
		sensorData.add_puck_sensors(0);
		*sensorData.mutable_last_updated() = TimeUtil::SecondsToTimestamp(time(NULL) - startTime);

		size_t size = sensorData.ByteSizeLong();
		char* msg = new char [size];
		sensorData.SerializeToArray(msg, size);

		send(new_socket, msg, size, 0);
		close(new_socket);
		memset(buffer,0,128);
		delete[] msg;
	}

	void* status;
	for (int i = 0; i < 2; i++) {
		pthread_join (threads[i], &status);
	}

	destroyAllWindows();
	return 0;
}
