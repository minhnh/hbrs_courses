#ifndef GREEDY_SEARCH_HPP
#define GREEDY_SEARCH_HPP
#include "state.hpp"

enum Direction
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3
};


class Greedy_search
{
    public:
        Greedy_search();
        void run();
        void search_h1(int map[]);
        void search_h2(int map[]);
        State move(int map[], Direction);
        bool compare_arrays(int a[], int b[]);
        bool is_related(State a, State b);
        
        int size_x;
		int size_y;
    private:

};
#endif
