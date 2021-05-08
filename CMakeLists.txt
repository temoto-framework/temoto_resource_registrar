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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemotoRequestMetadata.msg"
  "msg/TemotoResponseMetadata.msg"
  "srv/LoadComponent.srv"
  "srv/StatusComponent.srv"
  "srv/UnloadComponent.srv"
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )

add_executable(publisher src/Publisher.cpp)
ament_target_dependencies(publisher rclcpp tutorial_interfaces)         # CHANGE

add_executable(subscriber src/Subscriber.cpp)
ament_target_dependencies(subscriber rclcpp tutorial_interfaces)     # CHANGE

add_executable(consumer src/ResourceConsumer.cpp)
ament_target_dependencies(consumer rclcpp tutorial_interfaces Boost rr_core)     # CHANGE

install(TARGETS
  publisher
  subscriber
  consumer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
