cmake_minimum_required(VERSION 3.10)
project(temoto_resource_registrar)

add_compile_options(-std=c++14 -Wno-dev)

option(TEMOTO_ENABLE_TRACING_ "Use tracer" OFF)
if(TEMOTO_ENABLE_TRACING_)
  add_compile_options(-Dtemoto_enable_tracing)
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  actionlib 
  actionlib_msgs
  std_msgs
  message_generation
  rosconsole_bridge
)

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
  DataFetchComponent.srv
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
  ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp-build/output/libopentracing.so
)

if(TEMOTO_ENABLE_TRACING_)
  list(APPEND headers
    ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp/example/tutorial
    ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp/3rd_party/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp-build/include
  )

  list(APPEND libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/opentracing-cpp/src/opentracing-cpp-build/output/libopentracing.so
    pthread
  )
endif()

catkin_package(
  INCLUDE_DIRS ${headers}
  LIBRARIES ${libraries}
  CATKIN_DEPENDS roscpp std_msgs message_generation actionlib_msgs rosconsole_bridge
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

add_custom_command(TARGET rr_core001
  POST_BUILD
  COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/build/librr_core.so.0.0.1 
    ${CATKIN_DEVEL_PREFIX}/lib/librr_core.so.0.0.1
)