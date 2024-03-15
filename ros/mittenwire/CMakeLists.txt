cmake_minimum_required(VERSION 3.0.2)

project(mittenwire)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -O3")

find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES "include" "libusb" "libusb-1.0")

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
  dynamic_reconfigure
)

add_message_files(
  DIRECTORY msg
  FILES
  Packet.msg
  GloveStatus.msg
  ImpedanceMatrix.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Hub.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
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
    dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIR}
  include
)
 
set(LIBRARY_NAME "${PROJECT_NAME}_core")
add_library(${LIBRARY_NAME} 
  src/camera.cpp
  src/hub.cpp
  src/imagepublisher.cpp
  src/master.cpp
  src/message.cpp
  src/node.cpp
  src/object.cpp
  src/packet.cpp
  src/superspeed.cpp
  src/utils.cpp
)
add_dependencies(${LIBRARY_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} usb-1.0)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
install(TARGETS ${LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_dependencies(${LIBRARY_NAME} ${PROJECT_NAME}_gencfg)

set(PYTHON_NAME "py${PROJECT_NAME}")
pybind_add_module(${PYTHON_NAME}
  src/python.cpp
)
target_link_libraries(${PYTHON_NAME} PRIVATE ${LIBRARY_NAME} ${catkin_LIBRARIES})

