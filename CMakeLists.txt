cmake_minimum_required(VERSION 3.5)
project(temoto_rr)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_library(rr_core REQUIRED)


find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

set(headers
  include
  ${Boost_INCLUDE_DIRS}
)
set(libraries
  ${Boost_LIBRARIES}
  rr_core
)
include_directories(
  ${headers}
)


# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(consumer src/ResourceConsumer.cpp)
target_link_libraries(consumer rr_core)
ament_target_dependencies(consumer rclcpp tutorial_interfaces Boost)     # CHANGE

add_executable(agent src/ResourceAgent.cpp)
target_link_libraries(agent rr_core)
ament_target_dependencies(agent rclcpp tutorial_interfaces Boost)     # CHANGE

install(TARGETS
  consumer
  agent
  DESTINATION lib/${PROJECT_NAME})

ament_package()
