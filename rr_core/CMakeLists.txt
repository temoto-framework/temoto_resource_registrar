cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(ExternalProject)

file(GLOB HEADERS
  "include/*.h"
)

file(GLOB SOURCES
  "src/*.cpp"
)

set(LIBRARIES)
set(tracing_dependencies)
set(LIBRARY_NAME
  rr_core
)

option(TEMOTO_ENABLE_TRACING_ "Use tracer" OFF)
if(TEMOTO_ENABLE_TRACING_)

  add_compile_options(-Dtemoto_enable_tracing)

  # Build OpenTracing 1.5.0
  ExternalProject_Add(opentracing-cpp
    URL https://github.com/opentracing/opentracing-cpp/archive/refs/tags/v1.5.0.tar.gz
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp
    BUILD_COMMAND $(MAKE) -j4
    INSTALL_COMMAND cmake -E echo "Skipping install step."
  )

  # Build jaeger-client-cpp 0.5.0
  ExternalProject_Add(jaeger-client-cpp
    URL https://github.com/jaegertracing/jaeger-client-cpp/archive/refs/tags/v0.5.0.tar.gz
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/jaeger-client-cpp
    BUILD_COMMAND $(MAKE) -j4
    INSTALL_COMMAND cmake -E echo "Skipping install step."
  )

  list(APPEND HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp/src/opentracing-cpp/include
    ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp/src/opentracing-cpp/example/tutorial
    ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp/src/opentracing-cpp/3rd_party/include
    ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp/src/opentracing-cpp-build/include
  )

  list(APPEND LIBRARIES
    ${CMAKE_CURRENT_BINARY_DIR}/opentracing-cpp/src/opentracing-cpp-build/output/libopentracing.so
    pthread
  )
  list(APPEND tracing_dependencies
    opentracing-cpp
  )
endif()

include_directories(
  include
  ${HEADERS}
)

project(${LIBRARY_NAME} 
  LANGUAGES CXX 
  VERSION 0.0.1 
  DESCRIPTION "temoto rr core library"
)

add_library(${LIBRARY_NAME} SHARED
  ${SOURCES}
)

if(TEMOTO_ENABLE_TRACING_)
  add_dependencies(${LIBRARY_NAME}
    ${tracing_dependencies}
  )
endif()

target_link_libraries(${LIBRARY_NAME}
  ${LIBRARIES}
)

set_target_properties(${LIBRARY_NAME} PROPERTIES 
  VERSION ${PROJECT_VERSION}
  LINKER_LANGUAGE CXX
)

include(GNUInstallDirs)

set(SOURCE_FILES ${SOURCES})

configure_file(${LIBRARY_NAME}.pc.in ${LIBRARY_NAME}.pc @ONLY)

find_package (Boost 1.65.1 COMPONENTS system filesystem serialization REQUIRED)
if (Boost_FOUND)
  include_directories (SYSTEM ${Boost_INCLUDE_DIR})
endif ()

find_package(console_bridge)

# ExternalProject_Add(
#   console_bridge
#   URL https://github.com/ros/console_bridge/archive/master.zip
#   PREFIX ${CMAKE_CURRENT_BINARY_DIR}/console_bridge
#   INSTALL_COMMAND ""
#   LOG_DOWNLOAD ON
#   LOG_INSTALL ON
# )

# ExternalProject_Get_Property(console_bridge source_dir binary_dir)

# set(CONSOLE_BRIDGE_LIB_INCLUDE "${source_dir}/include")
# file(MAKE_DIRECTORY ${CONSOLE_BRIDGE_LIB_INCLUDE})
# set(CONSOLE_BRIDGE_BIN_LIB_INCLUDE "${binary_dir}")
# file(MAKE_DIRECTORY ${CONSOLE_BRIDGE_BIN_LIB_INCLUDE})

# list(APPEND LIB_INCLUDE_DIRS ${CONSOLE_BRIDGE_LIB_INCLUDE})
# list(APPEND LIB_INCLUDE_DIRS ${CONSOLE_BRIDGE_BIN_LIB_INCLUDE})

# add_library(libconsole_bridge SHARED IMPORTED)
# add_dependencies(libconsole_bridge console_bridge)
# set_target_properties(libconsole_bridge PROPERTIES IMPORTED_LOCATION ${binary_dir}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}console_bridge${CMAKE_SHARED_LIBRARY_SUFFIX})
# set_target_properties(libconsole_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${LIB_INCLUDE_DIRS}")

#############
# test flag
#############
option(test "Build all tests." OFF)


#############
# test config
#############
if (test)
  
  ExternalProject_Add(
    glog
    URL https://github.com/google/glog/archive/master.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog
    INSTALL_COMMAND ""
  )

  ExternalProject_Get_Property(glog source_dir binary_dir)


  set(GLOG_LIB_INCLUDE "${source_dir}/src")
  file(MAKE_DIRECTORY ${GLOG_LIB_INCLUDE})
  set(GLOG_BIN_LIB_INCLUDE "${binary_dir}")
  file(MAKE_DIRECTORY ${GLOG_BIN_LIB_INCLUDE})

  list(APPEND GLOG_LIB_INCLUDE_DIRS ${GLOG_LIB_INCLUDE})
  list(APPEND GLOG_LIB_INCLUDE_DIRS ${GLOG_BIN_LIB_INCLUDE})

  add_library(libglog SHARED IMPORTED)
  add_dependencies(libglog glog)
  set_target_properties(libglog PROPERTIES IMPORTED_LOCATION ${binary_dir}/${CMAKE_SHARED_LIBRARY_PREFIX}glog${CMAKE_SHARED_LIBRARY_SUFFIX} )
  set_target_properties(libglog PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GLOG_LIB_INCLUDE_DIRS}")

  find_package(Threads REQUIRED)

  ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/master.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    INSTALL_COMMAND ""
  )

  ExternalProject_Get_Property(gtest source_dir binary_dir)

  add_library(libgtest IMPORTED STATIC GLOBAL)
  add_dependencies(libgtest gtest)

  set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/lib/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
  )

  add_library(libgmock IMPORTED STATIC GLOBAL)
  add_dependencies(libgmock gtest)

  set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/lib/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
  )

  include_directories("${source_dir}/googletest/include"
                    "${source_dir}/googlemock/include")




  #enable_testing()
  #find_package(GTest)

  set(TEST_BINARY ${CMAKE_PROJECT_NAME}_test)
  file(GLOB_RECURSE TEST_SOURCES test/*.h test/*.cpp)

  add_executable(${TEST_BINARY} ${TEST_SOURCES})
  target_link_libraries(${TEST_BINARY} libgtest libgmock libglog console_bridge ${LIBRARY_NAME} ${Boost_LIBRARIES})

  #gtest_discover_tests(${TEST_BINARY}
  #  WORKING_DIRECTORY ${PROJECT_DIR}
  #)
  
endif()
#############



target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src
)

target_link_libraries(${LIBRARY_NAME} console_bridge)

install(TARGETS ${LIBRARY_NAME} EXPORT ${LIBRARY_NAME}-config
  ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${LIBRARY_NAME}-config DESTINATION share/cmake)

export(TARGETS ${PROJECT_NAME} FILE ${LIBRARY_NAME}-config.cmake)
