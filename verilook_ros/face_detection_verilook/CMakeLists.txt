cmake_minimum_required(VERSION 2.8.3)
project(face_detection_verilook)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    std_msgs
	${catkin_BUILD_PACKAGES}
)

catkin_package()

## Additional header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## CPP executables
add_executable(${PROJECT_NAME}
    src/face_detection_verilook.cpp
)

## Linkers
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
