//============================================================================
// Name        : Salesman.cpp
// Author      : Evgeniya
// Version     :
// Copyright   : all rights reserved
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include "salesman.hpp"

Salesman::Salesman(ifstream & in_file)
{
    cities = readFile(in_file);
    best_full_distance = fullDist(cities);
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

void Salesman::print_cities(vector<City> cities) {
    for (int i = 0; i < cities.size(); i++) {
        cout << cities[i].getName() << " " << cities[i].getXCoord() << " "
                << cities[i].getYCoord() << endl;
    }
}

float Salesman::distance(City city1, City city2) {
    return sqrt(
            pow((city1.getXCoord() - city2.getXCoord()), 2)
                    + pow((city1.getYCoord() - city2.getYCoord()), 2));
}

float Salesman::fullDist(vector<City> cities) {
    float dist = distance(cities[0], cities[cities.size() - 1]);
    for (int i = 0; i < (cities.size() - 1); i++) {
        dist += distance(cities[i], cities[i + 1]);
    }
    return dist;
}

/* Calculate distance around 2 cities to see if a swap should be done */
bool Salesman::should_swap(vector<City> cities, int i, int j) {
    if (compare_successor_node_value(cities, i, j) > 0.0)
        return true;
    else
        return false;
}

/* Calculate distance around 2 cities and return the value difference*/
float Salesman::compare_successor_node_value(vector<City> cities, int i, int j) {
    int front_i = (i > 0) ? i - 1 : cities.size() - 1;
    int front_j = (j > 0) ? j - 1 : cities.size() - 1;
    int back_i = (i < cities.size() - 1) ? i + 1 : 0;
    int back_j = (j < cities.size() - 1) ? j + 1 : 0;
    float before_swap, after_swap;

    if (back_i == j) {
        // special case 1: j is right after i
        before_swap = distance(cities[front_i], cities[i]) +
                      distance(cities[j], cities[back_j]);
        after_swap =  distance(cities[front_i], cities[j]) +
                      distance(cities[i], cities[back_j]);
    } else if (front_i == j) {
        // special case 2: i is right after j
        before_swap = distance(cities[front_j], cities[j]) +
                      distance(cities[i], cities[back_i]);
        after_swap =  distance(cities[front_j], cities[i]) +
                      distance(cities[j], cities[back_i]);
    } else {
        // general case
        before_swap = distance(cities[front_i], cities[i]) +
                      distance(cities[i], cities[back_i]) +
                      distance(cities[front_j], cities[j]) +
                      distance(cities[j], cities[back_j]);

        after_swap  = distance(cities[front_i], cities[j]) +
                      distance(cities[j], cities[back_i]) +
                      distance(cities[front_j], cities[i]) +
                      distance(cities[i], cities[back_j]);
    }
    //The smaller the distance, the higher the value of a node
    //The returned value is the delta_E in the pseudo code
	return before_swap - after_swap;
}

/* Implement Hill Climbing algorithm */
vector<City> Salesman::hillClimb(vector<City> cities_in) {
    vector<City> cities(cities_in);
    for (int i = 0; i < cities.size(); i++) {
        for (int j = 0; j < cities.size(); j++) {
            // swap only if not same city and should_swap() returns true
            if (i != j && should_swap(cities, i, j)) {
                swap(cities[i], cities[j]);
            }
        }
    }
    return cities;
}

void Salesman::random_restart_hill_climb() {

    cout <<  "Initial distance: " << best_full_distance << endl << endl;

    float nxt_full_dist = 0.0f;
    int hill_climb_cnt = 1;
    while (true) {
        srand(unsigned(time(0)));
        random_shuffle(cities.begin(), cities.end());

        cities = hillClimb(cities);

        nxt_full_dist = fullDist(cities);

        cout << "Distance after hill climb " << hill_climb_cnt << ": " <<
                nxt_full_dist << endl << endl;

        if (nxt_full_dist >= best_full_distance)
        {
            cout << "Reached local maximum of random restart." << endl;
            break;
        }
        else
        {
            best_full_distance = nxt_full_dist;
        }
        hill_climb_cnt++;
    }
    cout << "Best full distance found: " << best_full_distance << endl;
}

void Salesman::simulated_annealing(double minute)
{
	srand (time(NULL));

	float full_dist = fullDist(cities);
	float shortest_dist = 17000;
	float chance = 0.0;
	float delta_E = 0.0;
	int i = 0;
	int j = 0;
	double dice = 0.0;

	//Timing variables
	//Duration is in second
	float duration = minute * 60;
	float T = 1;
	float T_old = 1;
	float scheduler = 0.0;
	auto start = chrono::steady_clock::now();
	auto now = chrono::steady_clock::now();
	auto diff = now - start;

	while(duration - T > 0)
	{
		//Select a random successor node
		do
		{
			i = rand() % cities.size();
			j = rand() % cities.size();
		}
		while (i == j);

		//Compare value of the current node and the next node
		//Shorter distance means higher delta_E
		delta_E = compare_successor_node_value(cities, i, j);

		//Get amount of time left.
		now = chrono::steady_clock::now();
		T = chrono::duration_cast<chrono::seconds>(now - start).count();
		scheduler = (10 * pow(0.8, T));

		if (delta_E > 0)
		{
			//Distance of next node is lower than the current node
			swap(cities[i], cities[j]);
			full_dist = fullDist(cities);
		}
		else
		{
			//When the Value of next node is worse than current node
			//Role a dice and decide.
			chance = exp(delta_E / scheduler); 	   //Value from 0 to 1
			dice = double(rand()/RAND_MAX); //Value from 0 to 1
			if (dice < chance)
			{
				swap(cities[i], cities[j]);
				full_dist = fullDist(cities);
			}
		}


		//Result is only printed once per second.
		if (T_old != T)
		{
			printf("Distance: %8.2f - Time left: %.2f s \n",
							full_dist, duration - T);
		}
		T_old = T;


		if (full_dist < shortest_dist)
		{
			shortest_dist = full_dist;
		}
	}
	printf("Shortest distance: %8.2f - Time: %.2f s \n",
							shortest_dist, duration);

}
