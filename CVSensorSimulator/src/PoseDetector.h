/*
 * PoseDetector.h
 *
 *  Created on: Apr 15, 2019
 *      Author: calvin
 */

#ifndef POSEDETECTOR_H_
#define POSEDETECTOR_H_


#include "FrameBuffer.h"
#include "TaggedObject.h"
#include <math.h>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "apriltag/apriltag_pose.h"
}

enum class AngleUnit {
	RADIANS,
	DEGREES
};

class PoseDetector {
private:
	FrameBuffer* fb;
	apriltag_detection_info_t detInfo;
	std::vector<TaggedObject>* objects;

	cv::Mat frame;
	apriltag_family_t* tf;
	apriltag_detector_t* td;
	zarray_t* detections;

	int tagMatch(int tagID);
	pose2D pose3Dto2D(apriltag_pose_t pose, AngleUnit unit);
	void label_tag_detection(cv::Mat* frame, apriltag_detection_t* det);
public:
	PoseDetector(FrameBuffer* fb, apriltag_detection_info_t detInfo, std::vector<TaggedObject>* objects);
	~PoseDetector();
	void updatePoseEstimates();
	cv::Mat* getLabelledFrame();
};


#endif /* POSEDETECTOR_H_ */
