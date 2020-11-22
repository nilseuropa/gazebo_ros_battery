cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_battery)

  find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    message_generation
    gazebo_msgs
    roscpp
    nodelet
    std_srvs
    sensor_msgs
    rosconsole
    std_msgs
  )

  find_package(Boost REQUIRED COMPONENTS thread)

  include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  link_directories(
    ${catkin_LIBRARY_DIRS}
  )

  add_service_files(
    FILES
    Reset.srv
    SetCharge.srv
    SetTemperature.srv
  )

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS
    message_runtime
    gazebo_msgs
    roscpp
    nodelet
    std_srvs
    sensor_msgs
    rosconsole
    std_msgs
  )

###########
## Build ##
###########

add_library(gazebo_ros_battery src/gazebo_ros_battery.cpp)
target_link_libraries(gazebo_ros_battery gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  gazebo_ros_battery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
