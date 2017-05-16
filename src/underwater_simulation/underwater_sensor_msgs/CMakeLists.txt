cmake_minimum_required(VERSION 2.8.3)
project(underwater_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs visualization_msgs)

add_message_files(
  FILES
  DVL.msg
  Pressure.msg
)

add_service_files(
  FILES
  SpawnMarker.srv
)
                                        
generate_messages(
  DEPENDENCIES roscpp std_msgs visualization_msgs
) 

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp
)


