/*
 * city.h
 *
 *  Created on: Nov 20, 2015
 *      Author: elhe
 */

#ifndef _ASSIGNMENT09_CITY_H_
#define _ASSIGNMENT09_CITY_H_

using namespace std;

class City {
    private:
        float x;
        float y;
        float deadline;
        float dist_from_start;
        float arrival_time = -1.0f;

    public:
        City(float x, float y, float deadline);
        ~City();
        float getDeadline();
        float getX();
        float getY();
        void setDistFromStart(float);
        float getDistFromStart();
        void setArrivalTime(float);
        void print();
};

#endif /* _ASSIGNMENT09_CITY_H_ */
