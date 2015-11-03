#include <fstream>
#include <iostream>
#include "greedy_search.hpp"

using namespace std;

Greedy_search::Greedy_search()
{
	
}

void Greedy_search::run()
{
	int tiles[][] = {{  1,  4,  8}, 
					 {  3,  6,  2}, 
					 {  0,  5,  7}};
	
	
					 
	
}


State::State(int Tiles[][])
{
	map = Tiles;
	map_size_x = sizeof(map);
	map_size_y = sizeof(map[0]);
	h1 = find_heuristics_1();
	h2 = find_heuristics_2();
}

//Manhattan distance
int State::find_heuristics_1()
{
	int heuristics = 0;
	for (int y = 0; y < map_size_y; y++)
	{
		for (int x = 0; x < map_size_x; x++)
		{
			int correct_x = ((map_size_x * map_size_y -1) - map[x][y]) 
									% map_size_y;
			int correct_y = ((map_size_x * map_size_y -1) - map[x][y]) 
									/ map_size_y;
			heuristics = heuristics 
							+ abs(correct_x - x) 
							+ abs(correct_y - y);
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
			correct_value = (map_size_x * map_size_y -1) 
								- (x + map_size_y * y);			
			if (map[x][y] == correct_value)
			{
				heuristics++;
			}
		}
	}
	return heuristics;
}
