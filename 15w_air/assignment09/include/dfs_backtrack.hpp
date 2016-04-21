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
        // 1 distance unit per time unit
        const int VELOCITY = 1;
        // variables
        vector<City> cities;
        int order_option;
        // functions
        vector<City> hillClimb(vector<City>);
        vector<City> randomNextSuccessor(vector<City>);
        float distance(City, City);
        void readFile(ifstream &);
        void sort();
        bool dfs_backtrack_re(vector<int> &assignment, vector<int> &domain, float &elapsed_time);

    public:
        DFSBacktrack(ifstream &, int);
        ~DFSBacktrack();
        void dfs_backtrack();
        void print_cities();
};

#endif // _ASSIGNMENT09_DFS_BACKTRACK_H_
