cmake_minimum_required(VERSION 3.8)
project(dalislam_to_hdmapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  rosbag
  pcl_conversions
)

find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

# LASzip library
add_subdirectory(src/3rdparty/LASzip)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src/3rdparty/LASzip/include
  src/3rdparty/LASzip/include/laszip
  src/3rdparty/LASzip/dll
)

add_executable(listener
  src/dalislam_odometry_subscriber.cpp
  src/laz_writer.cpp
)

target_link_libraries(listener
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  laszip
)

install(TARGETS listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
