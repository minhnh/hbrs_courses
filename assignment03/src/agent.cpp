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
    int index = 0;
	cout << "Please enter search method index: ";
	cin >> index;
	if ( index == 1)
	{
		bfs();
	}
	else
	{
		cout << "Invalid search index"<< endl;
	}
    
}

void Agent::print_map()
{
   cout << map;
}

int Agent::bfs()
{
	cout << "\033[2J"; //Clear screen before display
	cout << "\033[1;1H]"; //move Cursor to row 1 column 1
	cout << "Breath first search";
	
	int number_of_dust = 0;
	int current_node = 0;
	int check_node = 0;
	int stored_node = 1;
	int checked_node = 1;
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
			stored_node = stored_node + 1;
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		checked_node = checked_node + 1;
		
		
		//Check left point
		check_node = current_node - 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		checked_node = checked_node + 1;
		
		//Check above point
		check_node = current_node + map_Width + 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		checked_node = checked_node + 1;
		
		//Check below point
		check_node = current_node - map_Width - 1;
		if (map[check_node] == ' ')
		{
			searchTree.push(check_node);
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		else if ( map[check_node] == '*')
		{
			searchTree.push(check_node);
			number_of_dust = number_of_dust + 1;
			map[check_node] = '-';
			stored_node = stored_node + 1;
		}
		checked_node = checked_node + 1;
		cout << "\033[2;1H]"; //move Cursor to row 2 column 1
		cout << "Node in queue:" << searchTree.size()<<endl;
		cout << map << endl;
	}
	cout << "Number of dust :" << number_of_dust << endl;
	cout << "Stored node    :" << stored_node << endl;
	cout << "Checked node   :" << checked_node << endl;
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

