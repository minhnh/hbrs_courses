#include <iostream>
#include <fstream>
#include <chrono>
#include "search.hpp"

using namespace std;
using ms = chrono::milliseconds;

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
    cout << "(3) A* iterative"<<endl;
    cin >> input;
    if ( input == 1)
    {
        s = GREEDY;
    }
    else if (input == 2)
    {
        s = ASTAR;
    }
    else if (input == 3)
    {
        s = ASTARITER;
    }
    else
    {
        cout << "Invalid input"<< endl;
        return -1;
    }
    Search solver(s, h);

    // Adding iterations. Till the solution is reached.
    auto start = chrono::steady_clock::now();

    if (s == ASTARITER) {
        for(int i = 0; solver.found_solution != true ; i++){
                solver.depth = i;
                solver.run();
            }
    }
    else {
        solver.run();
    }

    auto end = chrono::steady_clock::now();

    auto diff = end - start;

    cout<<"Elapsed time is: "<< chrono::duration_cast<ms>(diff).count()<<" ms"<<endl;

    return 0;
}
