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
 * FrameBuffer stores two frames: the active frame and the inactive frame. The active frame is overwritten
 * each time updateFrame() is called. When getFrame() is called the active frame is returned and the active
 * frame and inactive frame switch roles. The inactive frame is not modified until getFrame is called again.
 * This allows the active frame to be updated continuously until it is needed, ensuring the caller of
 * getFrame() will receive the newest frame data.
 */
class FrameBuffer{
private:
	cv::VideoCapture cap;
	int activeIndex;
	std::mutex index_lock;
	cv::Mat frames[2];
	std::mutex frame_lock[2];

	/**
	 * Gets the opposite of the current active index.
	 *
	 * @return The opposite index of the current active index (if 0 then 1 and vice versa).
	 */
	int otherIndex();
public:
	FrameBuffer();

	/**
	 * @param settings Camera settings data to initialize the FrameBuffer.
	 */
	FrameBuffer(VidCapSettings settings);

	~FrameBuffer();

	/**
	 * Store the most recent camera frame data into the active frame.
	 */
	void updateFrame();

	/**
	 * Return the contents of the active frame (the most recent frame data). Set the inactive
	 * frame as the new active frame.
	 *
	 * @return Pointer to the output frame.
	 */
	cv::Mat *getFrame();
};


#endif /* FRAMEBUFFER_H_ */
