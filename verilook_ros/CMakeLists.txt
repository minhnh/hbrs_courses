cmake_minimum_required(VERSION 2.8.3)
project(verilook_ros)

set(CMAKE_BUILD_TYPE Debug)
set(NODELET_LIBRARY_NAME face_detection_verilook)

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

catkin_package(
    LIBRARIES
        face_detection_verilook
    CATKIN_DEPENDS
        nodelet
        roscpp
        std_msgs
)

add_executable(${PROJECT_NAME}
    ros/src/face_detection_verilook_node.cpp
)

## Create the nodelet tutorial library
add_library(face_detection_verilook
    ros/src/face_detection_verilook_nodelet.cpp
    ros/src/face_detection_verilook_node.cpp
)

target_link_libraries(${NODELET_LIBRARY_NAME}
    -lNBiometricClient
    -lNBiometrics
    -lNLicensing
    -lNCore
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
    -lNBiometricClient
    -lNBiometrics
    -lNLicensing
    -lNCore
    ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(${NODELET_LIBRARY_NAME}
        ${catkin_EXPORTED_LIBRARIES}
    )
endif()

## Mark the nodelet library for installations
install(TARGETS ${NODELET_LIBRARY_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
    FILES
        nodelet_face_detection_verilook.xml
        face_detection_nodelet.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    DIRECTORY
        ros/launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

# Checking convention
roslint_cpp()
roslint_add_test()

### TESTS
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(ros/launch)
endif()
