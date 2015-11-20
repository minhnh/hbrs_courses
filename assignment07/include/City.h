/*
 * city.h
 *
 *  Created on: Nov 20, 2015
 *      Author: elhe
 */

#include <string>
#ifndef CITY_H_
#define CITY_H_

using namespace std;

class City {

private:
	string name;
	float xCoord;
	float yCoord;

public:

	City() {
	}

	City(const City& city) {
		this->setName(city.getName());
		this->setXCoord(city.getXCoord());
		this->setYCoord(city.getYCoord());
	}

	City(string name, float xCoord, float yCoord) {
		this->setName(name);
		this->setXCoord(xCoord);
		this->setYCoord(yCoord);
	}

	const string& getName() const {
		return name;
	}

	void setName(const string& name) {
		this->name = name;
	}

	double getXCoord() const {
		return xCoord;
	}

	void setXCoord(float coord) {
		xCoord = coord;
	}

	double getYCoord() const {
		return yCoord;
	}

	void setYCoord(float coord) {
		yCoord = coord;
	}

};

#endif /* CITY_H_ */
