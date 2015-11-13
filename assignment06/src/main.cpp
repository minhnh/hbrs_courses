#include <iostream>
#include <fstream>
#include "search.hpp"
using namespace std;

int main(int arc, char* argv[])
{
    Heuristics h;
    Strategy s;
    int input;

    cout << "Please choose heuristics: "<<endl;
    cout << "(1) Manhattan distance"<<endl;
    cout << "(2) Misplaced tiles"<<endl;
    cin >> input;
    if ( input == 1)
    {
        h = MANHATTAN;
    }
    else if (input == 2)
    {
        h = MISPLACED;
    }
    else
    {
        cout << "Invalid input"<< endl;
        return -1;
    }
    cout << "Please choose search strategy: "<<endl;
    cout << "(1) Greedy"<<endl;
    cout << "(2) A*"<<endl;
    cin >> input;
    if ( input == 1)
    {
        s = GREEDY;
    }
    else if (input == 2)
    {
        s = ASTAR;
    }
    else
    {
        cout << "Invalid input"<< endl;
        return -1;
    }
    Search solver(s, h);
    solver.run();

    return 0;
}
