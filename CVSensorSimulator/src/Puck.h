/*
 * Puck.h
 *
 *  Created on: Apr. 19, 2019
 *      Author: calvin
 */

#ifndef PUCK_H_
#define PUCK_H_


#include "TaggedObject.h"

enum Color{	RED,
			GREEN,
			BLUE
};

class Puck : public TaggedObject {
protected:
	Color color;
	double radius;
public:
	Puck(int tagID, double radius, Color color);
	~Puck();
	double getRadius();
};


#endif /* PUCK_H_ */
