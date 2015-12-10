Go to the into the build folder and type:
    cmake ..

This will create the makefile for you following the rules in the CMakeLists.txt file.


Once the cmake command is done you can now compile your program with the following command:		make


The executable will be created inside the bin folder. Stay in the build folder and run the program with the following command:

    ./assignment09 <scenario_path> <order_option>   // Full command
    ./assignment09 <order_option>                   // Run scenario1.txt with specified order
    ./assignment09                                  // print usage message, run scenario1.txt, order mode is line number

    order_option possible values:
    1   - Line number
    2   - Euclidean distance
    3   - Time to deadline

Note: Before running the program put all needed text files (cities.txt, ten_cities.txt) into bin folder.
