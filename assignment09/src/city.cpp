#include <stdio.h>
#include "city.hpp"


City::City (float x, float y, float deadline) {
    this->x = x;
    this->y = y;
    this->deadline = deadline;
}

City::~City() {
}

float City::getX() {
    return x;
}

float City::getY() {
    return y;
}

float City::getDeadline() {
    return deadline;
}

void City::setDistFromStart(float dist_from_start) {
    this->dist_from_start = dist_from_start;
}

float City::getDistFromStart() {
    return this->dist_from_start;
}

void City::print() {
    printf( "X: %10f, Y: %10f, deadline: %10f\n", x, y, deadline);
}
