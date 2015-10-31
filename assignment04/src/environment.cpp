#include <fstream>
#include <iostream>
#include "environment.hpp"
#include "agent.hpp"

using namespace std;

Environment::Environment()
{
    map.clear();
    map_Height = 0;
    map_Width = 0;
    start_X = 0;
    start_Y = 0;
    number_of_dust = 0;
}

void Environment::run()
{
    int index = 0;
    cout << "Please enter map index: ";
    cin >> index;
    if ( load_map(index) != -1)
    {
        Agent robot(map);
        robot.get_map_data(map_Height, map_Width, start_X, start_Y,number_of_dust);
        robot.run();
    }
    else
    {
        cout << "Invalid map index"<< endl;
    }
}

int Environment::load_map(int map_index)
{
    ifstream mapFile;
    if (map_index == 1)
    {
        mapFile.open("../maps/map1.txt");
    }
    else if (map_index == 2)
    {
        mapFile.open("../maps/map2.txt");
    }
    else if (map_index == 3)
    {
        mapFile.open("../maps/map3.txt");
    }
    else
    {
        return -1;
    }

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

            if (c == 's')
            {
                start_X = map.length() - (height * (width + 1));
                start_Y = height;
            }
            else if (c >= 49 && c <= 57)
            {
                number_of_dust++;
            }

            map = map + c;
            c = mapFile.get();
        }
        map_Height = height;
        map_Width = width;
        cout << "H:" << map_Height << " W:" << map_Width <<endl;
    }
    else
    {
        cout << "Cannot open"<<endl;
        return -1;
    }

    // close the opened file.
    mapFile.close();


}

char Environment::get_value_at(int x, int y)
{
    //mapWidth + 1: map width does not count the \n at the end of each line
    if (x < map_Width && y < map_Height)
    {
        return map[x + y * (map_Width + 1)];
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

MapCell::MapCell(int X, int Y)
{
    MapCell::x = X;
    MapCell::y = Y;
}

MapCell::~MapCell()
{

}

void MapCell::set_x(int X)
{
    x = X;
}

void MapCell::set_y(int Y)
{
    y = Y;
}

void MapCell::set_xy(int X, int Y)
{
    x = X;
    y = Y;
}
