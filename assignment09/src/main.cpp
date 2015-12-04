#include <iostream>
#include <chrono>
#include "dfs_backtrack.hpp"

#define CITIES_FILE "scenarios/scenario1.txt"

using ms = chrono::milliseconds;

namespace {
    void usage() {
        cout << "Usage: ./assignment09 <scenario_path> <order_option>" << endl;
        cout << "       ./assignment09 <order_option>" << endl;
        cout << "       ./assignment09" << endl << endl;
        cout << "   order_option    -   Order of locations when beginning search" << endl;
        cout << "                       (default to 1):" << endl;
        cout << "                       1   - Line number" << endl;
        cout << "                       2   - Euclidean distance" << endl;
        cout << "                       3   - Time to deadline" << endl;
        cout << "   scenario_path   -   Path to scenario file" << endl;
        cout << "                       (default to scenarios/scenario1.txt):"
            << endl << endl;
    }
}

int main(int argc, char* argv[]) {
    const char* file_name;
    // Default order is line number
    int order_option = 1;

    // Argument check
    if (argc < 2) {
        file_name = CITIES_FILE;
    } else {
        // Get file name
        file_name = argv[1];
        // Check for order_option argument
        if (argc == 3)
        {
            order_option = stoi(argv[2]);
        }
        else
        {
            cout << "Too many arguments" << endl << endl;
            // Usage message
            usage();
            return 2;
        }
    }

    // Try to open file
    ifstream in_file(file_name);
    if (!in_file)
    {
        cout << "Can't open file" << endl;
        return -1;
    }
    DFSBacktrack dfs_backtrack = DFSBacktrack(in_file);

    // Close file instance
    in_file.close();

    // Adding iterations. Till the solution is reached.
    auto start = chrono::steady_clock::now();

    dfs_backtrack.dfs_backtrack();

    // some code here
    // to compute its execution duration in runtime
    auto end = chrono::steady_clock::now();

    auto diff = end - start;

    cout<<"Elapsed time is: "<< chrono::duration_cast<ms>(diff).count()<<" ms"<<endl;
    return 0;
}
