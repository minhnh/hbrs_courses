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
#include <stdio.h>
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
        cout << "City " << i + 1 << ": ";
        cities[i].print();
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

bool DFSBacktrack::dfs_backtrack_re(vector<int> &assignment,
        vector<int> &index_domain, float &elapsed_time) {

    /* Based on algorithm on page 13 slide chapter05.pdf */
    // Update arrival time of last city
    cities[assignment.back()].setArrivalTime(elapsed_time);
    // If all variables are assigned, return true
    if (assignment.size() == cities.size())
        return true;
    // variable = last element of assignment
    // iterate over possible values in index_domain
    for (vector<int>::iterator index = index_domain.begin();
            index != index_domain.end();
            ++index) {
        float dist = distance(cities[assignment.back()], cities[*index]);
        float arrival_time = elapsed_time + dist / VELOCITY;
        // Check constraint
        if (arrival_time < cities[*index].getDeadline()) {
            assignment.push_back(*index);
            index_domain.erase(index);
            // Recursive call to next assignment
            bool result = dfs_backtrack_re(assignment, index_domain, arrival_time);
            // Check result
            if (result) {
                return result;
            }
            // Remove value from assignment and add back to index_domain
            int index_value = assignment.back();
            assignment.pop_back();
            index_domain.insert(index, index_value);
        }
    }
    return false;
}

void DFSBacktrack::dfs_backtrack() {

    if (cities.size() < 2) {
        cout << "No cities in list" << endl;
        return;
    }

    sort();

    // Print starting sorted list
    cout << "Sorted city list:" << endl << endl;
    print_cities();
    cout << endl;

    // Setup initial assignment and domain
    vector<int> assignment;
    assignment.push_back(0);

    vector<int> index_domain;
    for (int i = 1; i < cities.size(); i++) {
        index_domain.push_back(i);
    }

    float elapsed_time = cities[0].getDeadline();

    // Call recursive function
    bool result = dfs_backtrack_re(assignment, index_domain, elapsed_time);

    // Print solution if found
    if (result) {
        cout << "Solution: " << endl << endl;
        for (int i = 0; i < assignment.size(); i++) {
            printf("City %2d: ", assignment[i] + 1);
            cities[assignment[i]].print();
        }
    } else {
        cout << "Could not find a solution" << endl;
    }

    return;
}
