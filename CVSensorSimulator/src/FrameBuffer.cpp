/*
 * FrameBuffer.cpp
 *
 *  Created on: Apr 5, 2019
 *      Author: calvin
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
