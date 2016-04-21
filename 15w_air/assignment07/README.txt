Go to the into the build folder and type:
	cmake ..

This will create the makefile for you following the rules in the CMakeLists.txt file.


Once the cmake command is done you can now compile your program with the following command:		make


The executable will be created inside the bin folder. Stay in the build folder and run the program with the following command:

        bin/assignment07 <path_to_file>
i.e:    bin/assignment07 bin/cities.txt

If no input file given will look for ten_cities.txt in the same folder the program is run from.
