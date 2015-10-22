#include <fstream>
#include <iostream>
#include "environment.hpp"
#include "agent.hpp"

using namespace std;

Environment::Environment()
{
	
}

void Environment::run()
{
    load_map(1);
}

int Environment::load_map(int map_index)
{
	ifstream mapFile;
	mapFile.open("bin/maps/map1.txt");
	char output;
	if (mapFile.is_open()) 
	{
		output = mapFile.get();
		while (!mapFile.eof()) 
		{
			cout<<output;
			output = mapFile.get();
		}
		cout << endl;
	}
	else
	{
		cout << "Cannot open"<<endl;
	}

	// close the opened file.
	mapFile.close();


}


void Environment::initialize_map()
{
   
}


void Environment::print_map()
{
    
}
