#include "city.hpp"


City::City (string name, float xCoord, float yCoord) {
    this->setName(name);
    this->setXCoord(xCoord);
    this->setYCoord(yCoord);
}

City::City(const City & city)
{
    this->setName(city.getName());
    this->setXCoord(city.getXCoord());
    this->setYCoord(city.getYCoord());
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

double City::getXCoord() const {
    return xCoord;
}

void City::setXCoord(float coord) {
    xCoord = coord;
}

double City::getYCoord() const {
    return yCoord;
}

void City::setYCoord(float coord) {
    yCoord = coord;
}
