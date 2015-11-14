#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "search.hpp"
#include "state.hpp"
#include <deque>

using namespace std;

Search::Search(Strategy strategy, Heuristics h)
{
    this->strategy = strategy;
    this->heuristics = h;
}
Search::~Search()
{

}

void Search::run()
{
    size_x = 3;
    size_y = 3;
    int tiles[] = {1,  4,  8,
                   3,  6,  2,
                   0,  5,  7};

    best_first_search(tiles);
}

void Search::evaluate_next_state(State & current_state, State & next_state, Direction d)
{
    next_state.depth = current_state.depth + 1;
    next_state.last_move = d;
    next_state.last_state = &current_state;
    //cout << "current_state.f: " << current_state.f <<
    //        ", current_state.h: " << current_state.h << endl;
    switch (this->strategy) {
        case ASTAR:
            next_state.f = next_state.h + next_state.depth;
            break;
        case GREEDY:
            next_state.f = next_state.h;
            break;
        default:
            next_state.f = INT_MAX;
            break;
    }
    //next_state.print();
}


void Search::best_first_search(int intput_map[])
{
    deque<State> fringe;
    deque<State> closed;
    deque<State> solve_step;
    int expanded_node = 0;

    // Setup initial state
    State initial_state(intput_map, size_x,size_y, this->heuristics);
    initial_state.f = initial_state.h;
    initial_state.depth = 0;
    fringe.push_front(initial_state);

    while (fringe.size() > 0)
    {
        State current_state =  fringe.front();
        fringe.pop_front();
        // Goal check
        if (current_state.h == 0)
        {
            solve_step.push_front(current_state);
            break;
        }
        // Expand states
        if(current_state.can_move_up)
        {
            expanded_node++;
            add_next_state(current_state, UP, fringe, closed);
        }
        if(current_state.can_move_down)
        {
            expanded_node++;
            add_next_state(current_state, DOWN, fringe, closed);
        }
        if(current_state.can_move_left)
        {
            expanded_node++;
            add_next_state(current_state, LEFT, fringe, closed);
        }
        if(current_state.can_move_right)
        {
            expanded_node++;
            add_next_state(current_state, RIGHT, fringe, closed);
        }
        // Push current state to closed
        closed.push_back(current_state);
    }

    // Backtracking solution steps
    while(solve_step.front().depth > 0)
    {
        int last_state_map[size_x * size_y];
        move(solve_step.front().map, last_state_map,
                                    revert_direction(solve_step.front().last_move));
        for (int i = 0; i < closed.size(); i++)
        {
            if (compare_arrays(last_state_map, closed[i].map))
            {
                solve_step.push_front(closed[i]);
                i == closed.size();
            }
        }
    }
    // Print solution steps
    for (int i = 0; i < solve_step.size(); i++)
    {
        solve_step[i].print();
    }
    cout<<"Steps taken:"<<solve_step.size()<<endl;
    cout<<"Expanded nodes:"<<expanded_node<<endl;
}

void Search::add_next_state(State &current_state, Direction d,
                                deque<State> &fringe,
                                deque<State> &closed)
{
    // Next state calculations
    int map[size_x * size_y];
    move(current_state.map, map, d);
    State next_state(map, size_x, size_y, this->heuristics);
    evaluate_next_state(current_state, next_state, d);

    // Check in closed for same state. Return if same state found.
    for (int i = 0; i < closed.size();i++)
    {
        if (compare_arrays(next_state.map, closed[i].map) &&
            next_state.f >= closed[i].f)
        {
            return;
        }
    }
    // Check in fringe for same state. If a same state in fringe has bigger
    // depth swap with next_state. Return if same state found.
    for (int i = 0; i < fringe.size();i++)
    {
        if (compare_arrays(next_state.map, fringe[i].map))
        {
            if (next_state.depth < fringe[i].depth)
                fringe[i] = next_state;
            return;
        }
    }
    // Add state to fringe in increasing order
    if (next_state.f > fringe.back().f)
    {
        fringe.push_back(next_state);
    }
    else
    {
        fringe.push_front(next_state);
        for (int i = 0; i+1 < fringe.size(); i++)
        {
            if (fringe[i].f > fringe[i+1].f)
            {
                State temp = fringe[i];
                fringe[i] = fringe[i+1];
                fringe[i+1] = temp;
            }
            else
            {
                break;
            }
        }
    }
}

void Search::move(int current_map[], int map[], Direction d)
{
    int zero_index = 0;
    for (int i = 0; i < size_x * size_y; i++)
    {
        map[i] = current_map[i];
        if (map[i] == 0)
        {
            zero_index = i;
        }
    }
    if (d == UP)
    {
        map[zero_index] = map[zero_index - size_y];
        map[zero_index - size_y] = 0;
    }
    else if (d == DOWN)
    {
        map[zero_index] = map[zero_index + size_y];
        map[zero_index + size_y] = 0;
    }
    else if (d == LEFT)
    {
        map[zero_index] = map[zero_index - 1];
        map[zero_index - 1] = 0;
    }
    else if (d == RIGHT)
    {
        map[zero_index] = map[zero_index + 1];
        map[zero_index + 1] = 0;
    }
}

bool Search::compare_arrays(int a[], int b[])
{
    for (int i = 0; i < sizeof(a); i++)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

Direction Search::revert_direction(Direction d)
{
    if (d == 0)
    {
        return DOWN;
    }
    else if (d == 1)
    {
        return UP;
    }
    else if (d == 2)
    {
        return RIGHT;
    }
    else if (d == 3)
    {
        return LEFT;
    }
    return UP;
}
