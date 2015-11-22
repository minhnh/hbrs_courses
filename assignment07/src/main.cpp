#include <iostream>
#include <ctime>
#include <algorithm>
#include "salesman.hpp"

#define CITIES_FILE "ten_cities.txt"

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

    clock_t startTime = clock();
    srand(unsigned(time(0)));
    float cur_full_dist = salesman.fullDist(cities);
    float nxt_full_dist = 0.0f;
    while (true) {
        random_shuffle(cities.begin(), cities.end());
        cout << "Initial list of cities" << endl;
        for (vector<int>::size_type i = 0; i != cities.size(); i++) {
            cout << cities[i].getName() << " " << cities[i].getXCoord() << " "
                << cities[i].getYCoord() << endl;
        }
        cout << endl << "Distance before " << salesman.fullDist(cities) << endl;
        cities = salesman.hillClimb(cities);
        nxt_full_dist = salesman.fullDist(cities);
        cout << endl << "List after hillClimb" << endl;
        for (vector<int>::size_type i = 0; i != cities.size(); i++) {
            cout << cities[i].getName() << " " << cities[i].getXCoord() << " "
                    << cities[i].getYCoord() << endl;
        }
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
    cout << double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." << endl;
    return 0;
}
