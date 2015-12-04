#include "city.hpp"


City::City (float x, float y, float deadline) {
    this->x = x;
    this->y = y;
    this->deadline = deadline;
}

City::~City()
{

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
