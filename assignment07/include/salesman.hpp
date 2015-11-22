#ifndef SALESMAN_H_
#define SALESMAN_H_

#include <vector>
#include <fstream>
#include "city.hpp"

using namespace std;

class Salesman {
    private:
        vector<City> cities;

        vector<City> hillClimb(vector<City> cities_in);
        float fullDist(vector<City> cities);
        float distance(City city1, City city2);
        vector<City> swap(vector<City> cities, int i, int j);
        vector<City> readFile(ifstream & in_file);

    public:
        Salesman(ifstream & in_file);
        ~Salesman();
        void random_restart_hill_climb();
        void print_cities(vector<City>);
};

#endif // SALESMAN_H_
