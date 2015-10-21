cmake_minimum_required(VERSION 2.8)

project( Assignmen03 )

set(CMAKE_BUILD_TYPE Debug)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )

include_directories (
    include
)

add_executable( assignmet03 
		src/main.cpp
		src/agent.cpp
		src/environment.cpp )

#target_link_libraries(assignmet03)
