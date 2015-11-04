#include <fstream>
#include <iostream>
#include <stdlib.h> 
#include "greedy_search.hpp"
#include "state.hpp"
#include <deque>

using namespace std;

Greedy_search::Greedy_search()
{
	
}

void Greedy_search::run()
{
	size_x = 3;
	size_y = 3;
	int tiles[] = {1,  4,  8, 
				   3,  6,  2, 
				   0,  5,  7};
				   
	int index = 0;

    cout << "Please choose heuristics: "<<endl;
    cout << "(1) Manhattan distance"<<endl;
    cout << "(2) Misplaced tiles"<<endl;
    cin >> index;
    if ( index == 1) 
    {
		search_h1(tiles);
    } 
    else if (index == 2) 
    {
		search_h2(tiles);
    } else {
        cout << "Invalid heuristics index"<< endl;
    }


}

void Greedy_search::search_h1(int intput_map[])
{
	int expanded_node = 0;
	State initial_state(intput_map,size_x,size_y);
	
	deque<State> search_list;	
	deque<State> reached_state;
	deque<State> solve_step;
	
	search_list.push_front(initial_state);
	State past_state = initial_state;
	
	
	while (search_list.size() > 0)
	{
		State current_state =  search_list.front();
		reached_state.push_back(current_state);
		search_list.pop_front();
		if (current_state.h1 == 0)
		{
			solve_step.push_front(current_state);
			break;
		}
		if(current_state.can_move_up)
		{
			expanded_node++;
			State next_state = move(current_state.map,UP);
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
				if (next_state.h1 < search_list.front().h1 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h1 > search_list.back().h1)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h1 > search_list[i+1].h1)
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
		
		if(current_state.can_move_down)
		{
			expanded_node++;
			State next_state = move(current_state.map,DOWN);
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
				if (next_state.h1 < search_list.front().h1 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h1 > search_list.back().h1)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h1 > search_list[i+1].h1)
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

		if(current_state.can_move_left)
		{
			expanded_node++;
			State next_state = move(current_state.map,LEFT);
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
				if (next_state.h1 < search_list.front().h1 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h1 > search_list.back().h1)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h1 > search_list[i+1].h1)
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
		
		if(current_state.can_move_right)
		{
			expanded_node++;
			State next_state = move(current_state.map,RIGHT);
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
				if (next_state.h1 < search_list.front().h1 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h1 > search_list.back().h1)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h1 > search_list[i+1].h1)
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
	}
	
	for (int i = reached_state.size()-1; i >= 0; i--)
	{
		if (is_related(solve_step.front(),reached_state[i]))
		{
			solve_step.push_front(reached_state[i]);
		}
	}
	
	for (int i = 0; i < solve_step.size(); i++)
	{
		solve_step[i].print();
	}
	
	cout<<"Expanded nodes:"<<expanded_node<<endl;
}

void Greedy_search::search_h2(int intput_map[])
{
	int expanded_node = 0;
	State initial_state(intput_map,size_x,size_y);
	
	deque<State> search_list;	
	deque<State> reached_state;
	deque<State> solve_step;
	
	search_list.push_front(initial_state);
	State past_state = initial_state;
	
	
	while (search_list.size() > 0)
	{
		State current_state =  search_list.front();
		reached_state.push_back(current_state);
		search_list.pop_front();
		if (current_state.h2 == 0)
		{
		   	solve_step.push_front(current_state);
			break;
		}
		if(current_state.can_move_up)
		{
			expanded_node++;
			State next_state = move(current_state.map,UP);
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
				if (next_state.h2 < search_list.front().h2 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h2 > search_list.back().h2)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h2 > search_list[i+1].h2)
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
		
		if(current_state.can_move_down)
		{
			expanded_node++;
			State next_state = move(current_state.map,DOWN);
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
				if (next_state.h2 < search_list.front().h2 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h2 > search_list.back().h2)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h2 > search_list[i+1].h2)
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

		if(current_state.can_move_left)
		{
			expanded_node++;
			State next_state = move(current_state.map,LEFT);
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
				if (next_state.h2 < search_list.front().h2 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h2 > search_list.back().h2)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h2 > search_list[i+1].h2)
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
		
		if(current_state.can_move_right)
		{
			expanded_node++;
			State next_state = move(current_state.map,RIGHT);
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
				if (next_state.h2 < search_list.front().h2 or search_list.size() == 0)
				{
					search_list.push_front(next_state);
				}
				else if (next_state.h2 > search_list.back().h2)
				{
					search_list.push_back(next_state);
				}
				else
				{
					search_list.push_front(next_state);
					for (int i = 0; i+1 < search_list.size(); i++)
					{
						if (search_list[i].h2 > search_list[i+1].h2)
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
	}
	
	for (int i = reached_state.size()-1; i >= 0; i--)
	{
		if (is_related(solve_step.front(),reached_state[i]))
		{
			solve_step.push_front(reached_state[i]);
		}
	}
	
	for (int i = 0; i < solve_step.size(); i++)
	{
		solve_step[i].print();
	}
	
	cout<<"Expanded nodes:"<<expanded_node<<endl;
}


State Greedy_search::move(int current_map[], Direction d)
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

bool Greedy_search::compare_arrays(int a[], int b[])
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

bool Greedy_search::is_related(State a, State b)
{
    bool correct_zero = false;
	int correct_tiles = 0;
	if (a.can_move_up)
	{
		if(b.map[a.zero_index - size_y] == 0)
		{
			correct_zero = true;
		}
	}
	
	if (a.can_move_down)
	{
		if(b.map[a.zero_index + size_y] == 0)
		{
			correct_zero = true;
		}
	}
	
	if (a.can_move_left)
	{
		if(b.map[a.zero_index - 1] == 0)
		{
			correct_zero = true;
		}
	}
	
	if (a.can_move_right)
	{
		if(b.map[a.zero_index + 1] == 0)
		{
			correct_zero = true;
		}
	}
	
	for (int i = 0; i < size_x*size_y; i++)
	{
	    if (a.map[i] == b.map[i])
	    {
	        correct_tiles++;
	    }
	}
	
	if (correct_tiles == 7 and correct_zero)
	{
	   return true;
	}
	
	return false;
}
