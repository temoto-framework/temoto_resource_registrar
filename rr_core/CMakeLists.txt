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