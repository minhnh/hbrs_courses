//============================================================================
// Name        : Salesman.cpp
// Author      : Evgeniya
// Version     :
// Copyright   : all rights reserved
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include "salesman.hpp"

Salesman::Salesman()
{

}

Salesman::~Salesman()
{

}

vector<City> Salesman::readFile(ifstream & in_file) {

    string fileLine;
    string delimiter(",");
    vector<string> token;
    vector<City> cities;
    string cityName;
    float x, y;
    int number = -1;

    while (getline(in_file, fileLine)) {
        number++;
        size_t pos = fileLine.find(delimiter);
        do {
            token.push_back(fileLine.substr(0, pos));
            fileLine.erase(0, pos + delimiter.length());
            pos = fileLine.find(delimiter);

        } while (pos != string::npos);

        token.push_back(fileLine);

        if (number != 0) {
            cityName = token[0];
            x = atof(token[1].c_str());
            y = atof(token[2].c_str());
            City newCity(cityName, x, y);
            cities.push_back(newCity);
            //cout << cityName << " " << x << " " << y << " " << number << endl;
        }
        token.clear();
    }
    return cities;
}

vector<City> Salesman::swap(vector<City> cities, int i, int j) {
    //deep copy
    City temp(cities[i]);
    cities[i] = City(cities[j]);
    cities[j] = City(temp);
    return cities;
}

float Salesman::distance(City city1, City city2) {
    return sqrt(
            pow((city1.getXCoord() - city2.getXCoord()), 2)
                    + pow((city1.getYCoord() - city2.getYCoord()), 2));
}

float Salesman::fullDist(vector<City> cities) {
    float dist = distance(cities[0], cities[cities.size() - 1]);
    for (vector<int>::size_type i = 0; i < (cities.size() - 1); i++) {
        dist += distance(cities[i], cities[i + 1]);
    }
    return dist;
}

vector<City> Salesman::hillClimb(vector<City> cities_in) {
    vector<City> cities(cities_in);
    for (vector<City>::size_type i = 0; i != cities.size(); i++) {

        for (vector<City>::size_type j = 0; j != cities.size(); j++) {

            float distInit = fullDist(cities);

            if (i != j) {
                cities = swap(cities, i, j);
            }

            float distChanged = fullDist(cities);

            if (distChanged > distInit) {
                cities = swap(cities, i, j);
            } else {
                //cout << distChanged << endl;
            }
        }
    }
    return cities;
}
