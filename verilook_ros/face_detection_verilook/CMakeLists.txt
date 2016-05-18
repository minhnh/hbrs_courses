cmake_minimum_required(VERSION 2.8.3)
project(face_detection_verilook)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nodelet
    std_msgs
    roslint
	${catkin_BUILD_PACKAGES}
)

catkin_package()

## Additional header files
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
)

## CPP executables
add_executable(${PROJECT_NAME}
    ros/src/face_detection_verilook_node.cpp
    ros/src/face_detection_verilook_nodelet.cpp
)

## Linkers
target_link_libraries(${PROJECT_NAME}
    -lNBiometricClient
    -lNBiometrics
    -lNLicensing
    -lNCore
    ${catkin_LIBRARIES}
)

# Checking convention
roslint_cpp()
roslint_add_test()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
