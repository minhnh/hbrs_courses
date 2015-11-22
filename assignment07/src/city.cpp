#include "city.hpp"


City::City (string name, float xCoord, float yCoord) {
    this->setName(name);
    this->setXCoord(xCoord);
    this->setYCoord(yCoord);
}

City::~City()
{

}

const string& City::getName() {
    return name;
}

void City::setName(const string& name) {
    this->name = name;
}

double City::getXCoord() {
    return xCoord;
}

void City::setXCoord(float coord) {
    xCoord = coord;
}

double City::getYCoord() {
    return yCoord;
}

void City::setYCoord(float coord) {
    yCoord = coord;
}
