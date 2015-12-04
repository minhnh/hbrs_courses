#include "city.hpp"


City::City (float x, float y, float deadline) {
    this->setX(x);
    this->setY(y);
}

City::City(const City & city)
{
    this->setName(city.getName());
    this->setX(city.getX());
    this->setY(city.getY());
}

City::~City()
{

}

const string& City::getName() const {
    return name;
}

void City::setName(const string& name) {
    this->name = name;
}

double City::getX() const {
    return x;
}

void City::setX(float coord) {
    x = coord;
}

double City::getY() const {
    return y;
}

void City::setY(float coord) {
    y = coord;
}
