#include <stdio.h>
#include <iostream>
#include <vector>
#include "state.hpp"


State::State(std::vector<int> input_map, int x, int y)
{
    size_x = x;
    size_y = y;
    map = input_map;
    ultility = 0;
    is_terminal = false;
}

State::~State()
{
}

/* Ultility criteria:
 * 2 Points line:                           10 points
 * 3 Points line:                           50 points
 * 4 Points line:                           5000 points
 * Blocked 2 point line:                    -8 points
 * Blocked 3 point line:                    -48 points
 * Line with open end:                      5 points (for each open)
 * Point that block 3 points line         : 400 points
 * Point that block 2 points line         : 45 points
 */
void State::calculate_ultility()
{
    int Max = 0;
    int Min = 0;
    int points = 0;
    int cell = EMPTY;
    int neighbour_cell = EMPTY;
    int line_length = 0;
    for (int m = 0; m < size_x; m++)
    {
        for (int n = 0; n < size_y; n++)
        {
            cell = get_value_at(m,n);
            if (cell == X || cell == O)
            {
                points = 1;
                //Check all neighbour cell
                for (int i = -1; i <= 1 ; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        if (!(i == 0 && j == 0))
                        {
                            neighbour_cell = get_value_at(m + i,n + j);
                            //Neighbour is owned cell and the current cell
                            //is not already in a line.
                            if (neighbour_cell == cell &&
                                get_value_at(m - i,n - j) != cell)
                            {
                                //Check own line
                                line_length = 2;
                                while (get_value_at(m + line_length*i, n + line_length*j) == neighbour_cell)
                                {
                                    line_length++;
                                }

                                //Subtract point for creating a blocked line.
                                if (get_value_at(m + line_length*i, n + line_length*j) != EMPTY &&
                                    get_value_at(m - i, n - j) != EMPTY)
                                {
                                    if (line_length == 2)
                                    {
                                        points -= 8;
                                    }
                                    else if (line_length == 3)
                                    {
                                        points -= 48;
                                    }
                                }
                                //Add point for each open end
                                else 
                                {
									if (get_value_at(m + line_length*i, n + line_length*j) == EMPTY)
									{
										points += 5;
									}
									
									if (get_value_at(m - i, n - j) == EMPTY)
									{
										points += 5;
									}
                                    
                                }

                                //Add point for creating line
                                if (line_length == 2)
                                {
                                    points += 10;
                                }
                                else if (line_length == 3)
                                {
                                    points += 50;
                                }
                                else if (line_length > 3)
                                {
                                    is_terminal = true;
                                    points += 5000;
                                }
                            }
                            //Neighbour is oppenent's cell
                            else if (neighbour_cell == -cell)
                            {
                                //Check opponent's line
                                line_length = 2;
                                while (get_value_at(m + line_length*i, n + line_length*j) == neighbour_cell)
                                {
                                    line_length++;
                                }

                                //Current cell is not counted into length of opponent's line
                                line_length = line_length - 1;
                                //Add point for blocking opponent's line
                                if (line_length == 2)
                                {
                                    points += 300;
                                }
                                else if (line_length == 3)
                                {
                                    points += 400;
                                }
                            }
                        }
                    }
                }
                //Add points to Max or Min
                if (cell == X)
                {
                    Max = Max + points; 
                }
                else
                {
                    Min = Min + points;
                }
            }
        }
    }
    ultility = Max - Min;
}

bool State::is_terminal_state() {
    return is_terminal;
}

int State::get_ultility()
{
    calculate_ultility();
    return ultility;
}

int State::get_minimax()
{

    return minimax_value;
}

void State::set_minimax(int value)
{
    minimax_value = value;
}

int State::get_value_at(int x, int y)
{
    if (x < 0 || y < 0 || x >= size_x || y >= size_y)
    {
        return EDGE;
    }
    return map[x + y*size_x];
}

void State::set_value_at(int x, int y, int Symbol)
{
    if (x >= 0 && y >= 0 && x < size_x && y < size_y)
    {
        map[x + y*size_x] = Symbol;
    }
}

std::vector<int> State::get_map()
{
    return map;
}

void State::print_map()
{
    int symbol = 0;
    for (int i = 0; i < size_x; i++)
    {
        for (int j = 0; j < size_y; j++)
        {
            symbol = get_value_at(i,j);
            if (symbol == EMPTY)
            {
                std::cout << "| ";
            }
            else if (symbol == X)
            {
                std::cout << "|X";
            }
            else if (symbol == O)
            {
                std::cout << "|O";
            }
        }
        std::cout << "|\n";
    }
    std::cout<<std::endl;
}
