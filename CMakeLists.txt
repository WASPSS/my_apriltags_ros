cmake_minimum_required(VERSION 2.8.3)
project(my_apriltags_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  message_generation
  std_msgs
  cmake_modules
  nodelet
)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime std_msgs nodelet apriltags
  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(include  ${catkin_INCLUDE_DIRS})
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(my_april_detect_ros src/my_april_detect_ros.cpp)
target_link_libraries(my_april_detect_ros ${catkin_LIBRARIES})
add_dependencies(my_april_detect_ros my_apriltags_ros)

#############
## Install ##
#############


## Mark cpp header files for installation
install(DIRECTORY include/apriltags_ros
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
