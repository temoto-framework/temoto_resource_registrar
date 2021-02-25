cmake_minimum_required(VERSION 3.0.2)
project(temoto_resource_registrar)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

find_library(rr_core REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
  RequestResource.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(resource_producer src/ResourceProducer.cpp)

target_link_libraries(
  resource_producer
  ${catkin_LIBRARIES}
  rr_core
)

add_dependencies(
  resource_producer
  ${temoto_resource_registrar_EXPORTED_TARGETS}
)
