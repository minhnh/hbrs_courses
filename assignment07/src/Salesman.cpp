//============================================================================
// Name        : Salesman.cpp
// Author      : Evgeniya
// Version     :
// Copyright   : all rights reserved
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "City.h"

using namespace std;

vector<City> readFile(const char* fileName) {

	ifstream inFile(fileName);
	string fileLine;
	string delimiter(",");
	vector<string> token;
	vector<City> cities;
	string cityName;
	float x, y;
	int number = -1;

	while (getline(inFile, fileLine)) {
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

vector<City> swap (vector<City> cities, int i, int j) {
	//deep copy
	City temp(cities[i]);
	cities[i] = City(cities[j]);
	cities[j] = City(temp);
	return cities;
}

float distance (City city1, City city2) {
	return sqrt(pow((city1.getXCoord() - city2.getXCoord()),2) + pow((city1.getYCoord() - city2.getYCoord()),2));
}

float fullDist (vector<City> cities){
	float dist = distance(cities[0], cities[cities.size() - 1]);
	for (vector<int>::size_type i = 0; i < (cities.size() - 1); i++){
		dist += distance(cities[i], cities[i + 1]);
	}
	return dist;
}

int main() {
	vector<City> cities = readFile("ten_cities.txt");
	for (vector<int>::size_type i = 0; i != cities.size(); i++){
		cout << cities[i].getName() << " " << cities[i].getXCoord() << " " << cities[i].getYCoord() << endl;
	}
	cities = swap(cities, 0, 1);
	for (vector<int>::size_type i = 0; i != cities.size(); i++){
			cout << cities[i].getName() << " " << cities[i].getXCoord() << " " << cities[i].getYCoord() << endl;
	}
	cout << fullDist(cities) << endl;
	return 0;
}
