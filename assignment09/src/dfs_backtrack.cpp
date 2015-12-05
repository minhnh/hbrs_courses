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
#include "dfs_backtrack.hpp"

DFSBacktrack::DFSBacktrack(ifstream & in_file, int order_option)
{
    readFile(in_file);

    if (order_option < 1 || order_option > 3) {
        cout << "Invalid order option" << endl;
        exit(3);
    }

    this->order_option = order_option;
}

DFSBacktrack::~DFSBacktrack()
{

}

void DFSBacktrack::readFile(ifstream & in_file) {

    string fileLine;
    string delimiter(" ");
    vector<string> token;
    float x, y;
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
        newCity.setDistFromStart(0);
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
            float deadline = atof(token[2].c_str());
            City newCity(x, y, deadline);
            // Distance from starting position for sorting
            newCity.setDistFromStart(distance(newCity, cities[0]));
            cities.push_back(newCity);
        }
        token.clear();
    }
}

void DFSBacktrack::print_cities() {
    for (int i = 0; i < cities.size(); i++) {
        cout << "City " << i + 1 << ": "
            << cities[i].getX() << " " << cities[i].getY() << " "
            << cities[i].getDeadline() << endl;
    }
}

float DFSBacktrack::distance(City city1, City city2) {
    return sqrt(
            pow((city1.getX() - city2.getX()), 2)
                    + pow((city1.getY() - city2.getY()), 2));
}

void DFSBacktrack::sort() {
    // The list is already ordered by line numbers
    if (this->order_option == ORDER_LINENUM)
        return;

    // Sort list with bubble sort. Initialize i to end of list
    int i = cities.size() - 1;
    bool restart = false;
    float compare_val_cur, compare_val_prev;
    while (true) {
        // Pick value based on order_option
        if (this->order_option == ORDER_EUCLIDEAN) {
            compare_val_cur = cities[i].getDistFromStart();
            compare_val_prev = cities[i - 1].getDistFromStart();
        } else {
            compare_val_cur = cities[i].getDeadline();
            compare_val_prev = cities[i - 1].getDeadline();
        }

        // swap adjacent members if previous > current
        if (compare_val_prev > compare_val_cur) {
            swap(cities[i - 1], cities[i]);
            restart = true;
        }

        i--;

        // restart until no swap was done in the previous step
        if (i == 1) {
            if (restart) {
                restart = false;
                i = cities.size() - 1;
            }
            else {
                break;
            }
        }
    }
}

void DFSBacktrack::dfs_backtrack() {

    if (cities.size() < 2) {
        cout << "No cities in list" << endl;
        return;
    }

    sort();

    print_cities();
    return;
}
