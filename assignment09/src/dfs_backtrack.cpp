/*
 *  Name        :   dfs_backtrack.cpp
 *  Author      :   Evgenya
 *  Version     :
 *  Copyright   :   all rights reserved
 *  Description :   Implement backtracking depth first search to solve a time
 *                  constrained city traveling problem.
 *  Modified    :   2015/12/04 by Minh Nguyen
 */
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include "dfs_backtrack.hpp"

DFSBacktrack::DFSBacktrack(ifstream & in_file, int order_option)
{
    readFile(in_file);
    this->order_option = order_option;
}

DFSBacktrack::~DFSBacktrack()
{

}

void DFSBacktrack::readFile(ifstream & in_file) {

    string fileLine;
    string delimiter(" ");
    vector<string> token;
    float x, y, deadline;
    int number = -1;
    int pos;

    // Read initial position
    if (getline(in_file, fileLine)) {
        // Read initial x
        pos = fileLine.find(delimiter);
        x = atof(fileLine.substr(0, pos).c_str());
        fileLine.erase(0, pos + delimiter.length());
        // Read initial y
        pos = fileLine.find(delimiter);
        y = atof(fileLine.substr(0, pos).c_str());
        fileLine.erase(0, pos + delimiter.length());
        // Add starting location to list
        City newCity(x, y, 0);
        cities.push_back(newCity);
    } else {
        cout << "Empty file" << endl;
        exit(3);
    }

    // Read rest of file
    while (getline(in_file, fileLine)) {
        number++;
        pos = fileLine.find(delimiter);
        do {
            token.push_back(fileLine.substr(0, pos));
            fileLine.erase(0, pos + delimiter.length());
            pos = fileLine.find(delimiter);
        } while (pos != string::npos);

        token.push_back(fileLine);

        if (number != 0) {
            x = atof(token[0].c_str());
            y = atof(token[1].c_str());
            deadline = atof(token[2].c_str());
            City newCity(x, y, deadline);
            cities.push_back(newCity);
            //cout << cityName << " " << x << " " << y << " " << number << endl;
        }
        token.clear();
    }
}

void DFSBacktrack::print_cities() {
    for (int i = 0; i < cities.size(); i++) {
        cout << "City " << i + 1 << ": " << cities[i].getX() << " "
                << cities[i].getY() << " " << cities[i].getDeadline() << endl;
    }
}

float DFSBacktrack::distance(City city1, City city2) {
    return sqrt(
            pow((city1.getX() - city2.getX()), 2)
                    + pow((city1.getY() - city2.getY()), 2));
}

void DFSBacktrack::dfs_backtrack() {
    print_cities();
    return;
}
