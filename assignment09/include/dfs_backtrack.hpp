#ifndef _ASSIGNMENT09_DFS_BACKTRACK_H_
#define _ASSIGNMENT09_DFS_BACKTRACK_H_

#include <vector>
#include <fstream>
#include "city.hpp"

using namespace std;

class DFSBacktrack {
    private:
        vector<City> cities;
        int order_option;

        vector<City> hillClimb(vector<City> cities_in);
        vector<City> randomNextSuccessor(vector<City> cities_in);
        float distance(City city1, City city2);
        void readFile(ifstream & in_file);

    public:
        DFSBacktrack(ifstream & in_file, int order_option);
        ~DFSBacktrack();
        void dfs_backtrack();
        void print_cities();
};

#endif // _ASSIGNMENT09_DFS_BACKTRACK_H_
