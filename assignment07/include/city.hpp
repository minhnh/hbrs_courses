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
    City(string name, float xCoord, float yCoord);
    ~City();
    const string& getName();
    void setName(const string& name);
    double getXCoord();
    void setXCoord(float coord);
    double getYCoord();
    void setYCoord(float coord);
};

#endif /* CITY_H_ */
