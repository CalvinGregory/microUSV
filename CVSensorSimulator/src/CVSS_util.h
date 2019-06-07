/*
 * CVSS_utils.h
 *
 *  Created on: Jun. 7, 2019
 *      Author: calvin
 */

#ifndef CVSS_UTIL_H_
#define CVSS_UTIL_H_


#include <vector>
#include "TaggedObject.h"

namespace CVSS_util {
	int tagMatch(std::vector<TaggedObject>* objects, int tagID);
}


#endif /* CVSS_UTIL_H_ */
