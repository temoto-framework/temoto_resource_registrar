cmake_minimum_required(VERSION 3.5)
project(temoto_resource_registrar)

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

add_library(rr_core)
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/src)
file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp)
target_sources(rr_core PRIVATE ${SOURCES})
target_include_directories(rr_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/rr_core/include/temoto_resource_registrar)

# find_library(rr_core REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem serialization thread)

set(headers
  include
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rr
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
find_package(rr_interfaces REQUIRED)

add_executable(consumer src/ResourceConsumer.cpp)
target_link_libraries(consumer rr_core)
ament_target_dependencies(consumer rclcpp rr_interfaces Boost)

add_executable(agent src/ResourceAgent.cpp)
target_link_libraries(agent rr_core)
ament_target_dependencies(agent rclcpp rr_interfaces Boost)

add_executable(producer src/ResourceProducer.cpp)
target_link_libraries(producer rr_core)
ament_target_dependencies(producer rclcpp rr_interfaces Boost)

install(TARGETS
  consumer
  agent
  producer
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME})

install(DIRECTORY include/rr/
  DESTINATION include/rr
  FILES_MATCHING PATTERN "*.h"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(NOT EXISTS "$ENV{HOME}/.temoto")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory "$ENV{HOME}/.temoto"
  )
endif()

ament_export_include_directories(include)

ament_package()
