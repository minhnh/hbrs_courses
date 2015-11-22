#include <iostream>
#include <chrono>
#include <algorithm>
#include "salesman.hpp"

#define CITIES_FILE "ten_cities.txt"

using ms = chrono::milliseconds;

int main(int argc, char* argv[]) {
    vector<City> cities;
    const char* file_name;
    Salesman salesman = Salesman();

    // Extract file name from argument if exist, otherwise use default file
    // name
    if (argc == 1) {
        file_name = CITIES_FILE;
    }
    else
    {
        file_name = argv[1];
    }

    // Try to open file
    ifstream in_file(file_name);
    if (in_file)
    {
        cities = salesman.readFile(in_file);
    }
    else
    {
        cout << "Can't open default file, no file argument given" << endl;
        return -1;
    }

    // Adding iterations. Till the solution is reached.
    auto start = chrono::steady_clock::now();

    //TODO: Only calculate the cities around the swapping
    //TODO: check swapping from http://stackoverflow.com/questions/6224830/c-trying-to-swap-values-in-a-vector
    //TODO: should keep the starting city unchanged
    //TODO: Reduce prints for running with full cities.txt
    //TODO: Move this to salesman.cpp into a random_restart_hill_climb() function
    //TODO: Save initial list of cities as a variable
    float cur_full_dist = salesman.fullDist(cities);
    float nxt_full_dist = 0.0f;
    while (true) {
        srand(unsigned(time(0)));
        random_shuffle(cities.begin(), cities.end());

        cout << endl << "Distance before " << salesman.fullDist(cities) << endl;
        cities = salesman.hillClimb(cities);
        nxt_full_dist = salesman.fullDist(cities);

        cout << endl << "Distance after " << nxt_full_dist << endl << endl;
        if (nxt_full_dist >= cur_full_dist)
        {
            cout << "Reached local maximum of random restart." << endl;
            break;
        }
        else
        {
            cur_full_dist = nxt_full_dist;
        }
    }
    // some code here
    // to compute its execution duration in runtime
    auto end = chrono::steady_clock::now();

    auto diff = end - start;

    cout<<"Elapsed time is: "<< chrono::duration_cast<ms>(diff).count()<<" ms"<<endl;
    return 0;
}
