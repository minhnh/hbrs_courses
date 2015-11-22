#ifndef SALESMAN_H_
#define SALESMAN_H_

#include <vector>
#include <fstream>
#include "city.hpp"

using namespace std;

class Salesman {
    private:

    public:
        Salesman();
        ~Salesman();
        vector<City> hillClimb(vector<City> cities_in);
        void random_restart_hill_climb(vector<City> cities);
        float fullDist(vector<City> cities);
        float distance(City city1, City city2);
        vector<City> swap(vector<City> cities, int i, int j);
        vector<City> readFile(ifstream & in_file);
        void print_cities(vector<City>);
};

#endif // SALESMAN_H_
