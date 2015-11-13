#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "state.hpp"

using namespace std;

State::State(int Tiles[], int size_x, int size_y, Heuristics heuristics)
{
    for (int i = 0; i < size_x * size_y; i++)
    {
        map[i] = Tiles[i];
        if (map[i] == 0)
        {
            zero_index = i;
        }
    }
    map_size_x = size_x;
    map_size_y = size_y;

    f = INT_MAX;
    depth = INT_MAX;

    switch (heuristics)
    {
        case MANHATTAN:
            h = find_heuristics_1();
            break;
        case MISPLACED:
            h = find_heuristics_2();
            break;
        default:
            h = INT_MAX;
            break;
    }

    set_expandability();
}

State::~State()
{

}

void State::set_expandability()
{
    if (zero_index > map_size_y)
    {
        can_move_up = true;
    }
    else
    {
        can_move_up = false;
    }

    if (zero_index < map_size_x * (map_size_y -1))
    {
        can_move_down = true;
    }
    else
    {
        can_move_down = false;
    }

    if (zero_index % map_size_x != 0)
    {
        can_move_left = true;
    }
    else
    {
        can_move_left = false;
    }

    if (zero_index % map_size_x != (map_size_x -1))
    {
        can_move_right = true;
    }
    else
    {
        can_move_right = false;
    }
}


//Manhattan distance
int State::find_heuristics_1()
{
    int heuristics = 0;
    for (int y = 0; y < map_size_y; y++)
    {
        for (int x = 0; x < map_size_x; x++)
        {
            if (get_value_at(x,y) != 0)
            {
                int correct_x = (((map_size_x * map_size_y -1) - get_value_at(x,y))
                                        % map_size_y);
                int correct_y = (((map_size_x * map_size_y -1) - get_value_at(x,y))
                                        / map_size_y);
                heuristics = heuristics
                                + abs(correct_x - x)
                                + abs(correct_y - y);
            }
        }
    }
    return heuristics;
}

//Number of displaced tiles
int State::find_heuristics_2()
{
    int heuristics = 0;
    for (int y = 0; y < map_size_y; y++)
    {
        for (int x = 0; x < map_size_x; x++)
        {
            int correct_value = (map_size_x * map_size_y -1)
                                - (x + map_size_y * y);
            if (get_value_at(x,y) != correct_value
                    && get_value_at(x,y) != 0 )
            {
                heuristics++;
            }
        }
    }
    return heuristics;
}

void State::print()
{
    cout<<endl;
    for (int y = 0; y < map_size_y; y++)
    {
        for (int x = 0; x < map_size_x; x++)
        {
            int value = get_value_at(x,y);
            if (value == 0)
            {
                cout << " ";
            }
            else
            {
                cout << value;
            }
            if (x < 2)
            {
                cout << "|";
            }
        }
        cout << endl;
    }
}

int State::get_value_at(int x, int y)
{
    return map[x + y * map_size_y];
}

int State::get_index(int x, int y)
{
    return x + y * map_size_y;
}
