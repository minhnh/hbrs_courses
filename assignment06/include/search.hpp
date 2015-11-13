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

class Search
{
    public:
        Search();
        ~Search();
        void run();
        void search(int map[], int heuristic_id);
        State move(int map[], Direction);
        bool compare_arrays(int a[], int b[]);
        Direction revert_direction(int d);
        void insert_to_list(int heuristic_id, State &current_state, Direction d, std::deque<State> &search_list, std::deque<State> &reached_state);

        int size_x;
		int size_y;
    private:

};
#endif
