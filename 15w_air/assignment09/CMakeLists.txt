cmake_minimum_required(VERSION 2.8)

project( Assignment09 )

set(CMAKE_BUILD_TYPE Debug)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

include_directories (
    include
)

add_executable( assignment09
		src/dfs_backtrack.cpp
        src/city.cpp
        src/main.cpp)

# Add for chrono support
add_definitions(-std=c++11)

#target_link_libraries(assignment09)
