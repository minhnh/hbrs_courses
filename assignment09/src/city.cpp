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

void City::set_dist_from_start(float dist_from_start) {
    this->dist_from_start = dist_from_start;
}

float City::get_dist_from_start() {
    return this->dist_from_start;
}
