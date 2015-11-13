#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "search.hpp"
#include "state.hpp"
#include <deque>

using namespace std;

Search::Search(Strategy strategy)
{
    this->strategy = strategy;
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

    search(tiles);
}

void Search::search(int intput_map[])
{
    int expanded_node = 0;
    State initial_state(intput_map,size_x,size_y);

    deque<State> search_list;
    deque<State> reached_state;
    deque<State> solve_step;

    search_list.push_front(initial_state);
    initial_state.depth = 0;

    while (search_list.size() > 0)
    {
        State current_state =  search_list.front();
        reached_state.push_back(current_state);
        search_list.pop_front();
        if (current_state.h == 0)
        {
            solve_step.push_front(current_state);
            break;
        }
        if(current_state.can_move_up)
        {
            expanded_node++;
            insert_to_list(current_state, UP, search_list, reached_state);
        }

        if(current_state.can_move_down)
        {
            expanded_node++;
            insert_to_list(current_state, DOWN, search_list, reached_state);
        }

        if(current_state.can_move_left)
        {
            expanded_node++;
            insert_to_list(current_state, LEFT, search_list, reached_state);
        }

        if(current_state.can_move_right)
        {
            expanded_node++;
            insert_to_list(current_state, RIGHT, search_list, reached_state);
        }
    }

    while(solve_step.front().depth > 0)
    {
        State last_state = move(solve_step.front().map,
                                    revert_direction(solve_step.front().last_move));
        for (int i = 0; i < reached_state.size(); i++)
        {
            if (compare_arrays(last_state.map,reached_state[i].map))
            {
                solve_step.push_front(reached_state[i]);
                i == reached_state.size();
            }
        }
    }

    for (int i = 0; i < solve_step.size(); i++)
    {
        solve_step[i].print();
    }

    cout<<"Steps taken:"<<solve_step.size()<<endl;
    cout<<"Expanded nodes:"<<expanded_node<<endl;
}

void Search::insert_to_list(State &current_state, Direction d, std::deque<State> &search_list, std::deque<State> &reached_state)
{
    State next_state = move(current_state.map,d);

    if (this->strategy == GREEDY_MANHATTAN)
    {
        current_state.h = current_state.h1;
        next_state.h = next_state.h1;
    }
    else if (this->strategy == GREEDY_MISPLACED)
    {
        current_state.h = current_state.h2;
        next_state.h = next_state.h2;
    }
    next_state.depth = current_state.depth + 1;
    next_state.last_move = (int)d;
    bool state_reached = false;
    for (int i = 0; i < reached_state.size();i++)
    {
        if (compare_arrays(next_state.map,reached_state[i].map))
        {
            state_reached = true;
            break;
        }
    }
    if (!state_reached)
    {
        if (next_state.h < search_list.front().h or search_list.size() == 0)
        {
            search_list.push_front(next_state);
        }
        else if (next_state.h > search_list.back().h)
        {
            search_list.push_back(next_state);
        }
        else
        {
            search_list.push_front(next_state);
            for (int i = 0; i+1 < search_list.size(); i++)
            {
                if (search_list[i].h > search_list[i+1].h)
                {
                    State temp = search_list[i];
                    search_list[i] = search_list[i+1];
                    search_list[i+1] = temp;
                }
                else
                {
                    break;
                }
            }
        }
    }
}

State Search::move(int current_map[], Direction d)
{
    int map[size_x * size_y];
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
    State next_state(map, size_x, size_y);

    return next_state;
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

Direction Search::revert_direction(int d)
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
