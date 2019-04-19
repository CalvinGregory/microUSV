/*
 * Puck.h
 *
 *  Created on: Apr. 19, 2019
 *      Author: calvin
 */

#ifndef PUCK_H_
#define PUCK_H_


#include "TaggedObject.h"

class Puck : public TaggedObject {
protected:
	double radius;
public:
	Puck(int tagID, double radius);
	~Puck();
	double getRadius();
};


#endif /* PUCK_H_ */
