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
    temoto_resource_registrar
)

project(${LIBRARY_NAME} 
    LANGUAGES CXX 
    VERSION 0.0.1 
    DESCRIPTION "temodo library development"
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

find_package (Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
if (Boost_FOUND)
    include_directories (SYSTEM ${Boost_INCLUDE_DIR})
endif ()

#############
# test flag
#############
option(test "Build all tests." OFF)


#############
# test config
#############
if (test)
  include(ExternalProject)

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

  enable_testing()

  add_executable(rr_test test/rr_test.cpp)
  target_link_libraries(rr_test libgtest libgmock gtest_main)

  target_link_libraries(rr_test ${LIBRARY_NAME})

  add_test(NAME ResourceRegistrarTest COMMAND rr_test)
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
