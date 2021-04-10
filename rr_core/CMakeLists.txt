cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB HEADERS
    "include/*.h"
)

file(GLOB SOURCES
    "src/*.cpp"
)

set(LIBRARY_NAME
    rr_core
)

project(${LIBRARY_NAME} 
    LANGUAGES CXX 
    VERSION 0.0.1 
    DESCRIPTION "temodo rr core library"
)

add_library(${LIBRARY_NAME} SHARED
    ${SOURCES}
    ${HEADERS}
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

include(ExternalProject)

ExternalProject_Add(
  console_bridge
  URL https://github.com/ros/console_bridge/archive/master.zip
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/console_bridge
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(console_bridge source_dir binary_dir)

add_library(libconsole_bridge SHARED IMPORTED)
set_target_properties(libconsole_bridge PROPERTIES IMPORTED_LOCATION ${binary_dir}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}console_bridge${CMAKE_SHARED_LIBRARY_SUFFIX})
set_target_properties(libconsole_bridge PROPERTIES INCLUDE_DIRECTORIES ${source_dir}/include)

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

  add_library(libglog SHARED IMPORTED)
  set_target_properties(libglog PROPERTIES IMPORTED_LOCATION ${binary_dir}/${CMAKE_SHARED_LIBRARY_PREFIX}glog${CMAKE_SHARED_LIBRARY_SUFFIX} )


  include_directories(${source_dir}/src
                      ${binary_dir})

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
  target_link_libraries(${TEST_BINARY} libgtest libgmock libglog libconsole_bridge ${LIBRARY_NAME} ${Boost_LIBRARIES})

  #gtest_discover_tests(${TEST_BINARY}
  #  WORKING_DIRECTORY ${PROJECT_DIR}
  #)
  
endif()
#############



target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

install(TARGETS ${LIBRARY_NAME} EXPORT ${LIBRARY_NAME}-config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${LIBRARY_NAME}-config DESTINATION share/cmake)

export(TARGETS ${PROJECT_NAME} FILE ${LIBRARY_NAME}-config.cmake)
