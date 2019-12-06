/**
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

#ifndef FRAMEBUFFER_H_
#define FRAMEBUFFER_H_


#include <mutex>
#include <memory>
#include "opencv2/opencv.hpp"

/**
 * Struct defining relevant camera parameters.
 *
 * cameraID is the host computer's assigned camera ID value for the camera to be used.
 * x_res is the camera's x resolution.
 * y_res is the camera's y resolution.
 */
typedef struct {
	int cameraID;
	int x_res;
	int y_res;
} VidCapSettings;

/**
 * FrameBuffer is a class which captures and exports the most recent video frame from a camera.
 *
 * FrameBuffer is meant to be run in a loop calling updateFrame() in its own thread.
 *
 * FrameBuffer contains three frames: a reader frame, writer frame, and buffer frame. When updateFrame()
 * is called, a new frame is pulled from the camera into the writer frame. Once the frame grab is finished,
 * the new writer frame is moved into the frame buffer. When another thread calls getFrame(), if a new frame 
 * is available the buffer frame is moved into the reader frame which is then returned to the caller. 
 * 
 * The buffer frame is continuously overwritten to ensure the an eventual getFrame() call will receive the most
 * recent frame data. 
 */
class FrameBuffer{
private:
	cv::VideoCapture cap;
	std::shared_ptr<cv::Mat> readFrame;
	std::shared_ptr<cv::Mat> writeFrame;
	std::shared_ptr<cv::Mat> bufferFrame;
	std::mutex new_frame_lock;
	std::mutex buffer_lock;
public:
	FrameBuffer();

	/**
	 * @param settings Camera settings data to initialize the FrameBuffer.
	 */
	FrameBuffer(VidCapSettings settings);

	~FrameBuffer();

	/**
	 * Grabs a camera frame then stores it in bufferFrame.
	 */
	void updateFrame();

	/**
	 * Returns the most recently captured camera frame, i.e. the frame most recently stored in bufferFrame. 
	 * getFrame is a blocking operation. It will wait until a new frame is available in bufferFrame before returning.
	 *
	 * @return The output frame.
	 */
	cv::Mat getFrame();
};


#endif /* FRAMEBUFFER_H_ */
