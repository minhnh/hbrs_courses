#ifndef _ASSIGNMENT09_DFS_BACKTRACK_H_
#define _ASSIGNMENT09_DFS_BACKTRACK_H_

#include <vector>
#include <fstream>
#include "city.hpp"

using namespace std;

#define ORDER_LINENUM       1
#define ORDER_EUCLIDEAN     2
#define ORDER_DEADLINE      3

class DFSBacktrack {
    private:
        // variables
        vector<City> cities;
        int order_option;
        // functions
        vector<City> hillClimb(vector<City> cities_in);
        vector<City> randomNextSuccessor(vector<City> cities_in);
        float distance(City city1, City city2);
        void readFile(ifstream & in_file);
        void sort();

    public:
        DFSBacktrack(ifstream & in_file, int order_option);
        ~DFSBacktrack();
        void dfs_backtrack();
        void print_cities();
};

#endif // _ASSIGNMENT09_DFS_BACKTRACK_H_
