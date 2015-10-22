#include <iostream>
#include <queue>
#include "agent.hpp"

using namespace std;

Agent::Agent(string mapData)
{
	map = mapData;
}

Agent::~Agent()
{

}

void Agent::run()
{
    print_map();
    bfs();
    
}

void Agent::print_map()
{
   cout << map;
}

int Agent::bfs()
{
	int number_of_dust = 0;
	int current_node = 0;
	int check_node = 0;
	queue <int> searchTree;
	searchTree.push(get_index_at(start_X,start_Y));
	while(searchTree.size() > 0)
	{
		current_node = searchTree.front();
		searchTree.pop();
		map[current_node] = '-';
		
		//Check right point
		check_node = current_node + 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
		}
		
		
		//Check left point
		check_node = current_node - 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
		}
		
		//Check above point
		check_node = current_node + map_Width + 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
		}
		
		//Check below point
		check_node = current_node - map_Width - 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
		}
		cout << searchTree.size()<<endl;
		cout << map << endl;
	}
	cout << number_of_dust << endl;
}

int Agent::dfs()
{
    
}

void Agent::get_map_data(int mapH, int mapW, int sX, int sY)
{
	map_Height = mapH;
	map_Width = mapW;
	start_X = sX;
	start_Y = sY;
}

char Agent::get_value_at(int x, int y)
{
	//mapWidth + 1: map width does not count the \n at the end of each line
	if (x < map_Width && y < map_Height)
	{
		return map[x + y * (map_Width + 1)];
	}
	return -1;
}

int Agent::get_index_at(int x, int y)
{
	//mapWidth + 1: map width does not count the \n at the end of each line
	if (x < map_Width && y < map_Height)
	{
		return x + y * (map_Width + 1);
	}
	return -1;
}

