
cmake_minimum_required(VERSION 2.8.3)
project(ros_deep_learning)

# locate dependencies
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)


# define catkin package
catkin_package(
    CATKIN_DEPENDS roscpp image_transport sensor_msgs
)

# include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# enable c++11 (TensorRT requirement)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# inference nodes
add_executable(detectnet src/deep_learning_node.cpp src/node_detectnet.cpp)# src/image_converter.cpp)
target_link_libraries(detectnet ${catkin_LIBRARIES} )#jetson-inference)


#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()

