#ifndef SEARCH_HPP
#define SEARCH_HPP
#include <deque>
#include "state.hpp"

enum Strategy
{
    GREEDY = 0,
    ASTAR
};

class Search
{
    public:
        Search(Strategy, Heuristics);
        ~Search();
        void run();

        int size_x;
        int size_y;
    private:
        Strategy strategy;
        Heuristics heuristics;
        void search(int map[]);
        void move(int [], int [], Direction);
        bool compare_arrays(int a[], int b[]);
        Direction revert_direction(Direction d);
        void insert_to_list(State &, Direction, std::deque<State> &,
                                                std::deque<State> &);

        void evaluate_next_state(State &, State &, Direction);
};
#endif
