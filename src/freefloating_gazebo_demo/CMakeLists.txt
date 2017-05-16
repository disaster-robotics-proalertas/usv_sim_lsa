cmake_minimum_required(VERSION 2.8.3)
project(freefloating_gazebo_demo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
)

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# demo node
add_executable(freefloating_gazebo_demo src/freefloating_gazebo_demo.cpp)
target_link_libraries(freefloating_gazebo_demo ${catkin_LIBRARIES})
