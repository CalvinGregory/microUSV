/*
 * CVSS_utils.cpp
 *
 *  Created on: Jun. 7, 2019
 *      Author: calvin
 */

#include "CVSS_util.h"

namespace CVSS_util {

	int tagMatch(std::vector<TaggedObject>* objects, int tagID){
		int index = -1;
		for (uint i = 0; i < objects->size(); i++) {
			if (objects->at(i).getTagID() == tagID) {
				index = i;
				break;
			}
		}
		return index;
	}

}
