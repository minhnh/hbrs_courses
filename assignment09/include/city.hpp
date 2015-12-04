/*
 * city.h
 *
 *  Created on: Nov 20, 2015
 *      Author: elhe
 */

#ifndef _ASSIGNMENT09_CITY_H_
#define _ASSIGNMENT09_CITY_H_

#include <string>

using namespace std;

class City {
    private:
        string name;
        float xCoord;
        float yCoord;

    public:
        City(string name, float xCoord, float yCoord);
        City(const City & city);
        ~City();
        const string& getName() const;
        void setName(const string& name);
        double getXCoord() const;
        void setXCoord(float coord);
        double getYCoord() const;
        void setYCoord(float coord);
};

#endif /* _ASSIGNMENT09_CITY_H_ */
