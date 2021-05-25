cmake_minimum_required(VERSION 2.8.3)
project(grvc_e_clustering)

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
)



add_library(grvc_e_clustering
  src/MyCluster.cpp
  src/AEClustering.cpp
)

add_executable(test_grvc_e_clustering_node src/test_aeclustering.cpp)
target_link_libraries(test_grvc_e_clustering_node grvc_e_clustering ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES})
