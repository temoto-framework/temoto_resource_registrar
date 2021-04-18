cmake_minimum_required(VERSION 3.0.2)
project(temoto_resource_registrar)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  actionlib 
  actionlib_msgs
  std_msgs
  message_generation)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

#set(rr_core_DIR /usr/local/share/cmake)
#find_package(rr_core REQUIRED)

add_message_files(
  FILES
  TemotoRequestMetadata.msg
  TemotoResponseMetadata.msg
)

add_action_files(
  DIRECTORY action
  FILES
)

add_service_files(FILES
  LoadComponent.srv
  UnloadComponent.srv
  StatusComponent.srv
  CounterService.srv
)

generate_messages(DEPENDENCIES 
  actionlib_msgs
  std_msgs
)

include(ExternalProject)

ExternalProject_Add(rr_core001
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rr_core
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build
  CMAKE_ARGS
    -Wno-dev
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND cmake -E echo "Skipping install step."
)

set(headers
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/include
)

set(libraries
  ${Boost_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/librr_core.so.0.0.1
)

catkin_package(
  INCLUDE_DIRS ${headers}
  LIBRARIES ${libraries}
  CATKIN_DEPENDS roscpp std_msgs message_generation actionlib_msgs
  #DEPENDS
) 

include_directories(
  ${headers}
)

## producer start
add_executable(resource_producer 
  src/ResourceProducer.cpp
)

add_dependencies(resource_producer
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  rr_core001
)

target_link_libraries(resource_producer
  ${catkin_LIBRARIES}
  ${libraries}
)
## producer end

## consumer start
add_executable(resource_consumer 
  src/ResourceConsumer.cpp
)

add_dependencies(resource_consumer
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  rr_core001
)

target_link_libraries(resource_consumer
  ${catkin_LIBRARIES}
  ${libraries}
)

## consumer end

## agent start
add_executable(resource_agent 
  src/ResourceAgent.cpp
)

add_dependencies(resource_agent
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  rr_core001
)

target_link_libraries(resource_agent
  ${catkin_LIBRARIES}
  ${libraries}
)
## agent end

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/librr_core.so.0.0.1
  ${CATKIN_DEVEL_PREFIX}/lib/librr_core.so.0.0.1 COPYONLY
)