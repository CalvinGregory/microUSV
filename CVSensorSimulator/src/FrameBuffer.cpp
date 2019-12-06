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

#include "FrameBuffer.h"
#include <chrono>
using namespace cv;

FrameBuffer::FrameBuffer() {
	cap.open(0);
	if (!cap.isOpened())
		std::cerr << "Couldn't open video capture device" << std::endl;
	readFrame = std::make_shared<Mat>();
	writeFrame = std::make_shared<Mat>();
	bufferFrame = std::make_shared<Mat>();
	new_frame_lock.lock();
}

FrameBuffer::FrameBuffer(VidCapSettings settings) {
	cap.open(settings.cameraID);
	if (!cap.isOpened())
		std::cerr << "Couldn't open video capture device" << std::endl;
	cap.set(CAP_PROP_FRAME_WIDTH, settings.x_res);
	cap.set(CAP_PROP_FRAME_HEIGHT, settings.y_res);
	readFrame = std::make_shared<Mat>();
	writeFrame = std::make_shared<Mat>();
	bufferFrame = std::make_shared<Mat>();
	new_frame_lock.lock();
}

FrameBuffer::~FrameBuffer() {
	cap.release();
}

void FrameBuffer::updateFrame() {
	cap >> *writeFrame;
	std::lock_guard<std::mutex> lock(buffer_lock);
	writeFrame.swap(bufferFrame);
	new_frame_lock.try_lock();
	new_frame_lock.unlock();
}

Mat FrameBuffer::getFrame() {	
	new_frame_lock.lock();
	std::lock_guard<std::mutex> lock(buffer_lock);
	readFrame.swap(bufferFrame);

	return *readFrame;
}
