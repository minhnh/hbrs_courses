cmake_minimum_required(VERSION 2.8.3)
project(face_recognition_verilook)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    std_msgs
	${catkin_BUILD_PACKAGES}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES face_recognition_verilook
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(face_recognition_verilook
#   src/${PROJECT_NAME}/face_recognition_verilook.cpp
# )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(face_recognition_verilook_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Linkers
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
#############
## Install ##
#############
