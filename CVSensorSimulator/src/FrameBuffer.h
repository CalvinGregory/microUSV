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
using namespace cv;

typedef struct {
	int cameraID;
	int x_res;
	int y_res;
} VidCapSettings;

class FrameBuffer{
private:
	VideoCapture cap;
	int activeIndex;
	sem_t index_lock;
	Mat frames[2];
	sem_t frame_lock[2];
	int otherIndex();
public:
	FrameBuffer();
	FrameBuffer(VidCapSettings settings);
	~FrameBuffer();
	void updateFrame();
	Mat *getFrame();
};


#endif /* FRAMEBUFFER_H_ */
