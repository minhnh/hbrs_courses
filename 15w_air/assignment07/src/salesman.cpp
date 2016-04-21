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
float Salesman::swap_distance_change(vector<City> cities, int i, int j) {
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

    return (after_swap - before_swap);
}

/* Calculate distance around 2 cities to see if a swap should be done */
bool Salesman::should_swap(vector<City> cities, int i, int j) {
    if (swap_distance_change(cities, i, j) < 0.0)
        return true;
    else
        return false;
}

/* Implement Hill Climbing algorithm - old faster method */
vector<City> Salesman::wrongHillClimb(vector<City> cities_in) {
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

/* Implement Hill Climbing algorithm */
vector<City> Salesman::hillClimb(vector<City> cities_in) {
    vector<City> cities(cities_in);
    float min_change;

    do {
        int min_swap_index = -1;
        float change = 0.0f;
        // Change should be less than 0.0
        min_change = 0.0f;

        for (int i = 0; i < cities.size() - 1; i++) {
            // Calculate distance change
            change = swap_distance_change(cities, i, i + 1);
            // Update if distance is smaller
            if (change < min_change) {
                min_change = change;
                min_swap_index = i;
            }
        }

        // updated min_swap_index will be >= 0
        if (min_swap_index >= 0) {
            swap(cities[min_swap_index], cities[min_swap_index + 1]);
        }

    } while (min_change < 0.0f); // Stop if distance can't be decreased

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
