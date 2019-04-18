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
	sem_init(&index_lock, 0, 1);
	sem_init(&frame_lock[0], 0, 1);
	sem_init(&frame_lock[1], 0, 1);
}

FrameBuffer::FrameBuffer(VidCapSettings settings) {
	cap.open(settings.cameraID);
	if (!cap.isOpened())
		std::cerr << "Couldn't open video capture device" << std::endl;
	cap.set(CAP_PROP_FRAME_WIDTH, settings.x_res);
	cap.set(CAP_PROP_FRAME_HEIGHT, settings.y_res);
	activeIndex = 0;
	sem_init(&index_lock, 0, 1);
	sem_init(&frame_lock[0], 0, 1);
	sem_init(&frame_lock[1], 0, 1);
}

FrameBuffer::~FrameBuffer() {
	cap.release();
	sem_destroy(&index_lock);
	sem_destroy(&frame_lock[0]);
	sem_destroy(&frame_lock[1]);
}

void FrameBuffer::updateFrame() {
	int index;

	sem_wait(&index_lock);
	index = activeIndex;
	sem_wait(&frame_lock[index]);
	sem_post(&index_lock);

	cap >> frames[index];

	sem_post(&frame_lock[index]);
	return;
}

Mat *FrameBuffer::getFrame() {
	int index;
	Mat* frame;

	sem_wait(&index_lock);
	index = activeIndex;
	activeIndex = otherIndex();
	sem_wait(&frame_lock[index]);
	sem_post(&index_lock);

	frame = &frames[index];

	sem_post(&frame_lock[index]);
	return frame;
}

int FrameBuffer::otherIndex() {
	return abs(activeIndex - 1);
}
