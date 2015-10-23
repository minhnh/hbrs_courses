#include <iostream>
#include <queue>
#include <unistd.h>
#include <stdio.h>
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
    else if (index == 2)
    {
        dfs();
    }
	else
	{
		cout << "Invalid search index"<< endl;
	}
    
}

void Agent::print_map()
{
   cout << map << endl;
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
		if (number_of_dust >= max_number_of_dust)
		{
			break;
		}
	}
	cout << "Number of dust :" << number_of_dust << endl;
	cout << "Stored node    :" << stored_node << endl;
	cout << "Checked node   :" << checked_node << endl;
}

int Agent::dfs()
{
    cout << "\033[2J"; //Clear screen before display
	cout << "\033[1;1H]"; //move Cursor to row 1 column 1
	cout << "Depth first search";

	int number_of_dust = 0;
    int max_depth = 0;
    dfs_checked_node = 0;
    dfs_stored_node = 0;

	while ( number_of_dust < max_number_of_dust)
	{
		number_of_dust += dfs_re(start_X, start_Y, &max_depth);
    }
    cout << "Number of dust: " << number_of_dust << endl;
    cout << "Max Depth     : " << max_depth << endl;
    cout << "Stored node   : " << dfs_stored_node << endl;
    cout << "Checked node  : " << dfs_checked_node << endl;


    return number_of_dust;
}

int Agent::dfs_re(int x, int y, int * max_depth) {
    static int call_num = 0;
    int dust_num = 0;
    char value = get_value_at(x, y);
	dfs_checked_node ++;
    call_num++;
    if (call_num > *max_depth) {
        *max_depth = call_num;
    }

    if (value == 0 || value == '=' || value == '|' || value == '-') {
        call_num--;
        return dust_num;
    }

    cout << "\033[2;1H]"; //move Cursor to row 2 column 1
    printf("X:%4dY:%4d\n", x, y);
    set_value_at(x, y, 's');
    print_map();

    usleep(30000);
    set_value_at(x, y, value);

    if ( value == 's' || value == '*' || value == ' ' ) {
        set_value_at(x, y, '-');
        dust_num = (value == '*') ? 1 : 0;
        dust_num += dfs_re(x - 1, y, max_depth); // left
        dust_num += dfs_re(x + 1, y, max_depth); // right
        dust_num += dfs_re(x, y + 1, max_depth); // up
        dust_num += dfs_re(x, y - 1, max_depth); // down
        dfs_stored_node++;
    }

    call_num--;
    return dust_num;
}

void Agent::get_map_data(int mapH, int mapW, int sX, int sY, int nd)
{
	map_Height = mapH;
	map_Width = mapW;
	start_X = sX;
	start_Y = sY;
	max_number_of_dust = nd;
}

char Agent::get_value_at(int x, int y)
{
	//mapWidth + 1: map width does not count the \n at the end of each line
	if (x < map_Width && y < map_Height)
	{
		return map[x + y * (map_Width + 1)];
	}
	return 0;
}

char Agent::set_value_at(int x, int y, char value)
{
	//mapWidth + 1: map width does not count the \n at the end of each line
	if (x < map_Width && y < map_Height)
	{
		map[x + y * (map_Width + 1)] = value;
        return 1;
	}
	return 0;
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

