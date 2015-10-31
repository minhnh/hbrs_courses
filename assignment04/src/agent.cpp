#include <iostream>
#include <queue>
#include <unistd.h>
#include <stdio.h>
#include <ctime>
#include "environment.hpp"
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

	time_t start = time(NULL);
	ids();
	cout    << "Search time (s):"
			<< std::difftime(std::time(NULL), start) << endl;
}

void Agent::print_map()
{
    cout << map << endl;
}


/* Depth First Search Implementation */
int Agent::ids()
{
    cout << "\033[2J"; //Clear screen before display
    cout << "\033[1;1H]"; //move Cursor to row 1 column 1
    cout << "Depth first search";

    int number_of_dust = 0;
    int max_depth = 0;
    int depth_reached = 0;
    char target = '1';
    MapCell coordinates = MapCell(-1, -1);

    /* Recursive call to start depth first search */
    while (target <= max_number_of_dust + '0') {
        coordinates = ids_re(start_X, start_Y, max_depth, &depth_reached, target);
        if (coordinates.is_valid()) {
            target++;
            start_X = coordinates.x;
            start_Y = coordinates.y;
            max_depth = 0;
            //print_map();
            //printf("Goal %c: X = %3d Y = %3d\n", target, coordinates.x, coordinates.y);
        }
        ids_clear_map(start_X, start_Y);
        /* If depth reached is less than depth limit */
        if (depth_reached < max_depth) {
            cout << "Could not find" << target << endl;
            target++;
        }
        depth_reached = 0;
        max_depth++;
    }

    print_map();

    printf( "Coordinates goal %c    : %3d %3d\n", target, coordinates.x, coordinates.y);
    cout << "Number of goals        : " << max_number_of_dust << endl;
    cout << "Number of stored nodes : " << depth_reached << endl;
    cout << "Number of checked nodes: " << ids_checked_node << endl;

    return number_of_dust;
}

/* Implementation of Iterative Deepening Search
 * @param max_depth: maximum recursive depth recorded
 */
MapCell Agent::ids_re(int x, int y, int max_depth, int * depth_reached, char target) {
    static int cur_depth = 0;
    MapCell coordinates = MapCell(-1, -1);
    char value;

    /* Calculate maximum recursive depth reached */
    if (cur_depth > max_depth) {
        return coordinates;
    }

    value = get_value_at(x, y);

    /* Maximum depth reached by search algorithm */
    if (cur_depth > *depth_reached) {
        *depth_reached = cur_depth;
    }

    /* Return if current node is an obstacle / visited node/ wall */
    if (value != 's' && value != ' ' && value != target) {//value < '1' && value > '9') {
        return coordinates;
    }

    /* Print current coordinates */
    cout << "\033[2;1H]"; //move Cursor to row 2 column 1
    printf("X:%4d Y:%4d\n", x, y);
    set_value_at(x, y, 's');
    print_map();

    /* Sleep timer for visibility */
    //usleep(3000);

    /* Found space or dust (if not starting position), make recursive
     * calls to adjacent cells */
    set_value_at(x, y, '-');
    if (value == target) {
        coordinates.set_xy(x, y);
        return coordinates;
    }

    cur_depth++;

    coordinates = ids_re(x + 1, y, max_depth, depth_reached, target); // right
    if (coordinates.is_valid()) {
        cur_depth--;
        return coordinates;
    }
    coordinates = ids_re(x - 1, y, max_depth, depth_reached, target); // left
    if (coordinates.is_valid()) {
        cur_depth--;
        return coordinates;
    }
    coordinates = ids_re(x, y + 1, max_depth, depth_reached, target); // up
    if (coordinates.is_valid()) {
        cur_depth--;
        return coordinates;
    }
    coordinates = ids_re(x, y - 1, max_depth, depth_reached, target); // down

    cur_depth--;
    return coordinates;
}

void Agent::ids_clear_map(int x, int y) {
    static int cur_depth = 0;
    char value;

    value = get_value_at(x, y);

    if (value != '-' && value != 's') {
        return;
    }

    if (value == '-') set_value_at(x, y, ' ');

    cur_depth++;

    ids_clear_map(x + 1, y);
    ids_clear_map(x - 1, y);
    ids_clear_map(x, y + 1);
    ids_clear_map(x, y - 1);

    cur_depth--;
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

