cmake_minimum_required(VERSION 3.0.2)

project(glovewise)

set(CMAKE_CXX_FLAGS "-std=c++17 -g -O3 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pybind11_catkin
  message_generation
  message_runtime
  cv_bridge
  image_transport
  eigen_conversions
)

find_package(CGAL REQUIRED)

catkin_python_setup()

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    pybind11_catkin
    message_generation
    message_runtime
    cv_bridge
    image_transport
    eigen_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

set(PYTHON_NAME "py${PROJECT_NAME}")
pybind_add_module(${PYTHON_NAME}
  src/python.cpp
)
target_link_libraries(${PYTHON_NAME} PRIVATE ${LIBRARY_NAME} ${catkin_LIBRARIES} CGAL::CGAL)
