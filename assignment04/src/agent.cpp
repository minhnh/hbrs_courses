#include <iostream>
#include <queue>
#include <unistd.h>
#include <stdio.h>
#include <ctime>
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
	dfs();
	cout    << "Search time (s):"
			<< std::difftime(std::time(NULL), start) << endl;
}

void Agent::print_map()
{
   cout << map << endl;
}


/* Depth First Search Implementation */
int Agent::dfs()
{
    cout << "\033[2J"; //Clear screen before display
    cout << "\033[1;1H]"; //move Cursor to row 1 column 1
    cout << "Depth first search";

    int number_of_dust = 0;
    int max_depth = 5;
    dfs_checked_node = 0;

    /* Recursive call to start depth first search */
    ids_re(start_X, start_Y, max_depth, '1');

    cout << "Number of dust         : " << number_of_dust << endl;
    cout << "Number of stored nodes : " << max_depth << endl;
    cout << "Number of checked nodes: " << dfs_checked_node << endl;

    return number_of_dust;
}

/* Implementation of Iterative Deepening Search
 * @param max_depth: maximum recursive depth recorded
 */
void Agent::ids_re(int x, int y, int max_depth, char target) {
    static int cur_depth = 0;
    char value = get_value_at(x, y);

    dfs_checked_node ++;

    /* Calculate maximum recursive depth reached */
    cur_depth++;
    if (cur_depth > max_depth) {
        cur_depth--;
        return;
    }

    /* Return if current node is an obstacle / visited node/ wall */
    if (value != 's' && value != target && value != ' ') {
        cur_depth--;
        return;
    }

    /* Print current coordinates */
    cout << "\033[2;1H]"; //move Cursor to row 2 column 1
    printf("X:%4d Y:%4d\n", x, y);
    set_value_at(x, y, 's');
    print_map();

    /* Sleep timer for visibility */
    usleep(30000);
    set_value_at(x, y, value);

    /* Found space or dust (if not starting position), make recursive
     * calls to adjacent cells */
    set_value_at(x, y, '-');
    if (value == target) {
        return;
    }

    ids_re(x - 1, y, max_depth, target); // left
    ids_re(x + 1, y, max_depth, target); // right
    ids_re(x, y + 1, max_depth, target); // up
    ids_re(x, y - 1, max_depth, target); // down

    cur_depth--;
    return;
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

