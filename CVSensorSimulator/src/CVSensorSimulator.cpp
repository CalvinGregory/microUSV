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

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>
#include <functional>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <google/protobuf/util/time_util.h>

#include "FrameBuffer.h"
#include "PoseDetector.h"
#include "Robot.h"
#include "ConfigParser.h"
#include "CVSS_util.h"
#include "cyclicbarrier.hpp"
#include "CSVWriter.h"
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
ConfigParser::Config config;
Mat* frame;
Mat targetMask;

// Build barriers
class concrete_callable : public cbar::callable {
public:
	concrete_callable() {}
	virtual void run() override {}
};
auto cc = new concrete_callable();
cbar::cyclicbarrier* frameAcquisitionBarrier = new cbar::cyclicbarrier(2,cc);
cbar::cyclicbarrier* detectorBarrier0 = new cbar::cyclicbarrier(3,cc);
cbar::cyclicbarrier* detectorBarrier1 = new cbar::cyclicbarrier(3,cc);

/*
 * Video Capture thread function. Continuously updates the FrameBuffer.
 *
 * @param fb The FrameBuffer object to update.
 */
void video_capture_thread(FrameBuffer& fb) {
	while(running) {
		fb.updateFrame();
	}
}

/*
 * AprilTag detector thread function. Detects poses of AprilTags in the most
 * recent frame data from the FrameBuffer.
 *
 * @param pd The PoseDetector object which performs detections.
 */
void apriltag_detector_thread(PoseDetector& pd) {
	Mat temp_frame(100, 100, CV_8UC3, Scalar(0,0,0));
	imshow("CVSensorSimulator", temp_frame);

	while (running) {
		frameAcquisitionBarrier->await();
		pd.updatePoseEstimates(frame); 
		if(visualize) {
			Mat* labelledFrame = pd.getLabelledFrame(config);
			imshow("CVSensorSimulator", *labelledFrame);
		}
		// ESC key to exit
		if (waitKey(1) == 27) {
			running = false;
		}
		detectorBarrier0->await();
		detectorBarrier1->await();
	}
}

/*
 * Target detector thread function. Detects colored targets in the camera frame. 
 * Saves a mask image indicating which pixels are the targeted color. 
 * 
 * @param fb The FrameBuffer object to update.
 */
void target_detector_thread(FrameBuffer& fb) {
	Mat hsv;
	while (running) {
		frame = fb.getFrame();
		if (!frame->empty()) {
			cvtColor(*frame, hsv, COLOR_BGR2HSV);
		}
		frameAcquisitionBarrier->await();
		
		cv::inRange(hsv, config.target_thresh_low, config.target_thresh_high, targetMask);
		medianBlur(targetMask, targetMask, 7);
		
		detectorBarrier0->await();
		detectorBarrier1->await();
	}
}

/*
 * Thread function to process apriltag and target detections data. Generates simulated 
 * sensor values for target and obstacle detections to be sent to each microUSV and 
 * can export each vessel's pose history to a CSV file. 
 * 
 * @param taggedObjects List of all TaggedObjects being tracked by the simulator.
 * @param csv List of CSV file objects recording the pose of each robot.
 * @param output_csv Flag indicating if robot pose data should be recorded to the csv files. 
 */
void detection_processor_thread(vector<TaggedObject>& taggedObjects, vector<CSVWriter>& csv, bool output_csv) {
	Mat targets;
	while (running) {
		detectorBarrier0->await();
		detectorBarrier0->reset();

		vector<TaggedObject> robots(taggedObjects); 
		targets = targetMask.clone();

		detectorBarrier1->await(); 
		detectorBarrier1->reset();

		//TODO
		cout << "processor thread: calc sensor values" << endl;
		// Calc sensor values
		
		if (output_csv) {
			for(uint i = 0; i < csv.size(); i++) {
				pose2D pose = robots[i].getPose();
				csv[i].newRow() << pose.x << pose.y << pose.yaw;
			}
		}
	}

	// Cleanup barrier objects
	delete cc;
	delete frameAcquisitionBarrier;
	delete detectorBarrier0;
	delete detectorBarrier1;
}

int main(int argc, char* argv[]) {
//--- Initialize Threads ---//
	running = true;
	struct timeval startTime;
	gettimeofday(&startTime, NULL);

	// Parse config file and pull values into local variables.
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

	// int size = config.robots.size() + config.pucks.size();
	int size = config.robots.size(); // Current setup does not use pucks with apriltags, only the number of robots matters.
	vector<TaggedObject> taggedObjects(size);
	vector<CSVWriter> csv(config.robots.size());
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
	if (config.output_csv) {
		for (uint i = 0; i < csv.size(); i++) {
			csv[i].newRow() << "X" << "Y" << "Yaw";
		}
	}

	// Build thread parameter objects.
	FrameBuffer fb(settings);
	PoseDetector pd(info, &taggedObjects);
	Mat targetMask;

	// Start threads
	thread threads[4];
	threads[0] = thread(video_capture_thread, ref(fb));
	threads[1] = thread(apriltag_detector_thread, ref(pd));
	threads[2] = thread(target_detector_thread, ref(fb));
	threads[3] = thread(detection_processor_thread, ref(taggedObjects), ref(csv), config.output_csv);

	for (int i = 0; i < 4; i++) {
		threads[i].detach();
	}

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

		// Connect to microUSV and receive data.
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

			// Identify microUSV and respond with its sensor data.
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

	if (config.output_csv) {
		for (uint i = 0; i < csv.size(); i++) {
			stringstream fileName;
			fileName << taggedObjects[i].getLabel();
			fileName << "_pose_data_";
			auto t = std::time(nullptr);
			auto tm = *localtime(&t);
			fileName << put_time(&tm, "%Y-%m-%d_%H:%M");
			fileName << ".csv";
			csv[i].writeToFile(fileName.str());
		}
	}
	
	return 0;
}
