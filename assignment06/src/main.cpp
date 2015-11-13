#include <iostream>
#include <fstream>
#include "search.hpp"
using namespace std;

int main(int arc, char* argv[])
{
    Strategy s;
    int index;

    cout << "Please choose heuristics: "<<endl;
    cout << "(1) Manhattan distance"<<endl;
    cout << "(2) Misplaced tiles"<<endl;
    cin >> index;
    if ( index == 1)
    {
        s = GREEDY_MANHATTAN;
    }
    else if (index == 2)
    {
        s = GREEDY_MISPLACED;
    }
    else
    {
        cout << "Invalid heuristics index"<< endl;
        return -1;
    }
    Search solver(s);
    solver.run();

    return 0;
}
