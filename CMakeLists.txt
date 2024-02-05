cmake_minimum_required(VERSION 2.8.3)
project(clustering_with_ids)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dvs_msgs
  image_transport
  cv_bridge
)

catkin_package(CATKIN_DEPENDS std_msgs dvs_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/clustering_with_ids
  )

add_library(clustering_with_ids
  src/MyCluster.cpp
  src/AEClustering.cpp
  src/Tracking.cpp
  src/Improve.cpp
)

add_executable(clustering_with_ids_node src/id_speed_tracker.cpp)
target_link_libraries(clustering_with_ids_node clustering_with_ids ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
