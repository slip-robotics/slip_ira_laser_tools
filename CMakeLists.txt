cmake_minimum_required(VERSION 3.0.2)
project(slip_ira_laser_tools)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS laser_geometry roscpp sensor_msgs std_msgs tf dynamic_reconfigure pcl_ros)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
#  CATKIN_DEPENDS laser_geometry roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
target_link_libraries(laserscan_multi_merger ${catkin_LIBRARIES} ${PCL_LIBRARIES})

