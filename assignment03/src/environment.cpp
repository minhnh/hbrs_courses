#include <fstream>
#include <iostream>
#include "environment.hpp"
#include "agent.hpp"

using namespace std;

Environment::Environment()
{
	map.clear();
	mapHeight = 0;
	mapWidth = 0;
}

void Environment::run()
{
    load_map(1);
    print_map();
    cout << get_value_at(162,5) << endl;
    
}

int Environment::load_map(int map_index)
{
	ifstream mapFile;
	mapFile.open("bin/maps/map1.txt");
	char c;
	int width = 0;
	int height = 0;
	bool widthIsCalculated = false;
	if (mapFile.is_open()) 
	{
		c = mapFile.get();
		while (!mapFile.eof()) 
		{
			if (c == '\n')
			{
				if (!widthIsCalculated)
				{
					widthIsCalculated = true;
				}
				height = height + 1;
			}
			else
			{
				if (!widthIsCalculated)
				{
					width = width + 1;
				}
			}
			
			map = map + c;
			c = mapFile.get();
		}
		mapHeight = height;
		mapWidth = width;
		cout << "H:" << mapHeight << " W:" << mapWidth <<endl;
	}
	else
	{
		cout << "Cannot open"<<endl;
	}

	// close the opened file.
	mapFile.close();


}

char Environment::get_value_at(int x, int y)
{
	//mapWidth + 1: map width does not count the \n at the end of each line
	if (x < mapWidth && y < mapHeight)
	{
		return map[x + y * (mapWidth + 1)];
	}
	return -1;
}

void Environment::initialize_map()
{
   
}


void Environment::print_map()
{
    cout << map << endl;
}
