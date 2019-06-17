//============================================================================
// Name        : CVSensorSimulator.cpp
// Author      : CalvinGregory
// Version     : 1.0
// Copyright   : CERN Open Hardware Licence v1.2
// Description : Computer-Vision based sensor simulator for the microUSV
//============================================================================

#include <iostream>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>
#include <functional>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include <google/protobuf/util/time_util.h>

#include "FrameBuffer.h"
#include "PoseDetector.h"
#include "Robot.h"
#include "ConfigParser.h"
#include "CVSS_util.h"
#include "musv_msg.pb.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

using namespace std;
using namespace cv;
using google::protobuf::util::TimeUtil;

#define PORT 8078

bool running;
bool visualize;

void vid_cap_thread(FrameBuffer& fb) {
	while(running) {
		fb.updateFrame();
	}
}

void detector_thread(PoseDetector& pd) {

	Mat temp_frame(100, 100, CV_8UC3, Scalar(0,0,0));
	Mat* frame = &temp_frame;
	imshow("RobotDetections", *frame);

	while (running) {
		pd.updatePoseEstimates();
		if(visualize) {
			frame = pd.getLabelledFrame();
			imshow("RobotDetections", *frame);
		}
		// ESC key to exit
		if (waitKey(30) == 27) {
			running = false;
		}
	}
}

int main(int argc, char* argv[]) {
//--- Initialize Threads ---//
	running = true;
	struct timeval startTime;
	gettimeofday(&startTime, NULL);

	ConfigParser::Config config;
	if (argc > 1) {
		config = ConfigParser::getConfigs(argv[1]);
	}
	else {
		cerr << "No config file path provided." << endl;
		exit(-1);
	}
	visualize = config.visualize;

	apriltag_detection_info_t info;

	info.tagsize = config.tagsize;
	info.fx = config.cInfo.fx;
	info.fy = config.cInfo.fy;
	info.cx = config.cInfo.cx;
	info.cy = config.cInfo.cy;

	VidCapSettings settings;

	settings.cameraID = config.cInfo.cameraID;
	settings.x_res = config.cInfo.x_res;
	settings.y_res = config.cInfo.y_res;

	int size = config.robots.size() + config.pucks.size();
	vector<TaggedObject> taggedObjects(size);
	int i = 0;
	while (config.robots.size() > 0) {
		taggedObjects[i] = config.robots.front();
		config.robots.pop_front();
		i++;
	}
	while (config.pucks.size() > 0) {
		taggedObjects[i] = config.pucks.front();
		config.pucks.pop_front();
		i++;
	}

	FrameBuffer fb(settings);
	PoseDetector pd(&fb, info, &taggedObjects);

	thread threads[2];
	threads[0] = thread(vid_cap_thread, ref(fb));
	threads[1] = thread(detector_thread, ref(pd));

	threads[0].detach();
	threads[1].detach();

//--- Socket Setup --- //
	int server_fd, new_socket;
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

	// Forcefully attach socket to the port
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	// Make socket non-blocking
	fcntl(server_fd, F_SETFL, O_NONBLOCK);

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	// Forcefully attach socket to the port
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

//--- Request Receiver Loop ---//
	struct timeval currentTime;

	while (running) {
		mUSV::RequestData requestData;
		mUSV::SensorData sensorData;

		new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
		if(new_socket == -1) {
			if (errno != EWOULDBLOCK) {
				perror("error accepting connection");
				exit(EXIT_FAILURE);
			}
		}
		else {
			read(new_socket, buffer, 128);
			if (!requestData.ParseFromString(buffer)) {
				cerr << "Failed to parse Request Data message." << endl;
				return -1;
			}

			int index = CVSS_util::tagMatch(&taggedObjects, requestData.tag_id());
			pose2D pose = taggedObjects[index].getPose();

			currentTime = pose.timestamp;
			long seconds = currentTime.tv_sec - startTime.tv_sec;
			long uSeconds = currentTime.tv_usec - startTime.tv_usec;
			if (uSeconds < 0) {
				uSeconds = uSeconds + 1e6;
				seconds--;
			}

			sensorData.mutable_pose()->set_x(pose.x);
			sensorData.mutable_pose()->set_y(pose.y);
			sensorData.mutable_pose()->set_yaw(pose.yaw);
			sensorData.add_obstacle_sensors(0);
			sensorData.add_obstacle_sensors(1);
			sensorData.add_obstacle_sensors(2);
			sensorData.add_puck_sensors(3);
			sensorData.add_puck_sensors(4);
			sensorData.add_puck_sensors(5);
			*sensorData.mutable_timestamp() = TimeUtil::MicrosecondsToTimestamp(seconds * 1e6 + uSeconds);

			if(requestData.request_waypoints()) {
				for (std::list<ConfigParser::Waypoint>::iterator it = config.waypoints.begin(); it != config.waypoints.end(); it++) {
					mUSV::SensorData::Waypoint* waypoint = sensorData.add_waypoints();
					waypoint->set_x(it->x);
					waypoint->set_y(it->y);
				}

				sensorData.set_loop_waypoints(config.loop_waypoints);
			}

			size_t size = sensorData.ByteSizeLong();
			char* msg = new char [size];
			sensorData.SerializeToArray(msg, size);

			send(new_socket, msg, size, 0);
			close(new_socket);
			memset(buffer,0,128);
			delete[] msg;
		}
	}

	destroyAllWindows();
	return 0;
}
