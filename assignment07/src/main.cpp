#include <iostream>
#include <ctime>
#include <algorithm>
#include "salesman.hpp"

#define CITIES_FILE "ten_cities.txt"

int main(int argc, char* argv[]) {
    vector<City> cities;
    const char* file_name;
    Salesman salesman = Salesman();

    if (argc == 1) {
        file_name = CITIES_FILE;
    }
    else
    {
        file_name = argv[1];
    }

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
    random_shuffle(cities.begin(), cities.end());
    cities = salesman.swap(cities, 0, 1);
    cout << "Initial list of cities" << endl;
    for (vector<int>::size_type i = 0; i != cities.size(); i++) {
        cout << cities[i].getName() << " " << cities[i].getXCoord() << " "
            << cities[i].getYCoord() << endl;
    }
    cout << endl << "Distance before " << salesman.fullDist(cities) << endl;
    cities = salesman.hillClimb(cities);
    cout << endl << "Optimal list of cities" << endl;
    for (vector<int>::size_type i = 0; i != cities.size(); i++) {
        cout << cities[i].getName() << " " << cities[i].getXCoord() << " "
                << cities[i].getYCoord() << endl;
    }
    cout << endl << "Distance after " << salesman.fullDist(cities) << endl;
    // some code here
    // to compute its execution duration in runtime
    cout << double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." << endl;
    return 0;
}
