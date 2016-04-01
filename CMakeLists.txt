cmake_minimum_required(VERSION 2.8.7)

project(ros_ultrasonic_bumper_speed_filter)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# INCLUDES
set(ULTRASND_INC_PATH
    ${ros_ultrasonic_bumper_speed_filter_SOURCE_DIR}/src)

include_directories(${ULTRASND_INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE ULTRASND_SRC *.cpp)
# HEADERS
file(GLOB_RECURSE ULTRASND_HDR *.h)
#########################################################

#########################################################
# MSG FILES
# Adding MSG files to project source, without compiling them
file(GLOB_RECURSE MSG . *.msg)
set_source_files_properties(${MSG} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# LAUNCH FILES
# Adding LAUNCH files to project source, without compiling them
file(GLOB_RECURSE LAUNCH . *.launch)
set_source_files_properties(${LAUNCH} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
# OTHER FILES
# Adding other files to project source, without compiling them
file(GLOB_RECURSE OTHER . *.md)
set_source_files_properties(${OTHER} PROPERTIES HEADER_FILE_ONLY TRUE)
#########################################################

#########################################################
#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ros_ultrasonic_bumper
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    ros_ultrasonic_bumper
)

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    ultrasonic_bumper_speed_filter_node
    ${ULTRASND_HDR}
    ${ULTRASND_SRC}
    ${MSG}
    ${LAUNCH}
    ${OTHER}
)

target_link_libraries(
    ultrasonic_bumper_speed_filter_node
    ${catkin_LIBRARIES}
)
#########################################################
