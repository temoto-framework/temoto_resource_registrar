cmake_minimum_required(VERSION 3.0.2)
project(temoto_resource_registrar)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  actionlib 
  actionlib_msgs
  std_msgs
  message_generation)

find_package(Boost REQUIRED COMPONENTS system)

find_library(rr_core REQUIRED)

add_message_files(
  FILES
  TemotoMetadata.msg
)

add_action_files(
  DIRECTORY action
  FILES
)

add_service_files(
  FILES
  LoadComponent.srv
  UnloadComponent.srv
  StatusComponent.srv
  CounterService.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## producer start
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
## producer end

## consumer start
add_executable(resource_consumer src/ResourceConsumer.cpp)

target_link_libraries(
  resource_consumer
  ${catkin_LIBRARIES}
  rr_core
)

add_dependencies(
  resource_consumer
  ${temoto_resource_registrar_EXPORTED_TARGETS}
)
## consumer end

## agent start
add_executable(resource_agent src/ResourceAgent.cpp)

target_link_libraries(
  resource_agent
  ${catkin_LIBRARIES}
  rr_core
)

add_dependencies(
  resource_agent
  ${temoto_resource_registrar_EXPORTED_TARGETS}
)
## agent end