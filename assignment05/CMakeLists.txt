cmake_minimum_required(VERSION 2.8)

project( Assignment05 )

set(CMAKE_BUILD_TYPE Debug)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

include_directories (
    include
)

add_executable( assignment05 
		src/main.cpp
		src/greedy_search.cpp)

#target_link_libraries(assignment05)
