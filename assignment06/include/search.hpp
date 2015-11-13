#ifndef SEARCH_HPP
#define SEARCH_HPP
#include <deque>
#include "state.hpp"

enum Direction
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3
};

enum Strategy
{
    GREEDY_MANHATTAN = 0,
    GREEDY_MISPLACED,
    A_STAR_MANHATTAN,
    A_STAR_MISPLACED
};

class Search
{
    public:
        Search(Strategy);
        ~Search();
        void run();
        void search(int map[]);
        State move(int [], Direction);
        bool compare_arrays(int a[], int b[]);
        Direction revert_direction(int d);
        void insert_to_list(State &current_state, Direction d, std::deque<State> &search_list, std::deque<State> &reached_state);

        int size_x;
		int size_y;
    private:
        Strategy strategy;
};
#endif
