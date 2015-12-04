#include <iostream>
#include <chrono>
#include "dfs_backtrack.hpp"

#define CITIES_FILE "ten_cities.txt"

using ms = chrono::milliseconds;

int main(int argc, char* argv[]) {
    const char* file_name;
    double duration = 0.0;

    // Extract file name from argument if exist, otherwise use default file
    // name
    if (argc == 1)
    {
        file_name = CITIES_FILE;
    }
    else if (argc == 2)
    {
        file_name = argv[1];
        duration = 1;
    }
    else
    {
        file_name = argv[1];
        duration = std::stod(argv[2]);
    }

    // Try to open file
    ifstream in_file(file_name);
    if (!in_file)
    {
        cout << "Can't open default file, no file argument given" << endl;
        return -1;
    }
    Salesman salesman = Salesman(in_file);

    // Adding iterations. Till the solution is reached.
    auto start = chrono::steady_clock::now();

    salesman.simulated_annealing(duration);

    // some code here
    // to compute its execution duration in runtime
    auto end = chrono::steady_clock::now();

    auto diff = end - start;

    cout<<"Elapsed time is: "<< chrono::duration_cast<ms>(diff).count()<<" ms"<<endl;
    return 0;
}
