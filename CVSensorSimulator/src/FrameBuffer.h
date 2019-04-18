/*
 * FrameBuffer.h
 *
 *  Created on: Apr 5, 2019
 *      Author: calvin
 */

#ifndef FRAMEBUFFER_H_
#define FRAMEBUFFER_H_


#include "pthread.h"
#include "semaphore.h"
#include "opencv2/opencv.hpp"

typedef struct {
	int cameraID;
	int x_res;
	int y_res;
} VidCapSettings;

class FrameBuffer{
private:
	cv::VideoCapture cap;
	int activeIndex;
	sem_t index_lock;
	cv::Mat frames[2];
	sem_t frame_lock[2];
	int otherIndex();
public:
	FrameBuffer();
	FrameBuffer(VidCapSettings settings);
	~FrameBuffer();
	void updateFrame();
	cv::Mat *getFrame();
};


#endif /* FRAMEBUFFER_H_ */
