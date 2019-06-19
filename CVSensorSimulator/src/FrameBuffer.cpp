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
using namespace cv;

FrameBuffer::FrameBuffer() {
	cap.open(0);
	if (!cap.isOpened())
		std::cerr << "Couldn't open video capture device" << std::endl;
	activeIndex = 0;
}

FrameBuffer::FrameBuffer(VidCapSettings settings) {
	cap.open(settings.cameraID);
	if (!cap.isOpened())
		std::cerr << "Couldn't open video capture device" << std::endl;
	cap.set(CAP_PROP_FRAME_WIDTH, settings.x_res);
	cap.set(CAP_PROP_FRAME_HEIGHT, settings.y_res);
	activeIndex = 0;
}

FrameBuffer::~FrameBuffer() {
	cap.release();
}

void FrameBuffer::updateFrame() {
	int index;

	index_lock.lock();
	index = activeIndex;
	std::lock_guard<std::mutex> lock(frame_lock[index]);
	index_lock.unlock();

	cap >> frames[index];
}

Mat *FrameBuffer::getFrame() {
	int index;
	Mat* frame;

	index_lock.lock();
	index = activeIndex;
	activeIndex = otherIndex();
	std::lock_guard<std::mutex> lock(frame_lock[index]);
	index_lock.unlock();

	frame = &frames[index];
	return frame;
}

int FrameBuffer::otherIndex() {
	return abs(activeIndex - 1);
}
