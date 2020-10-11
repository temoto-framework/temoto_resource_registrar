cmake_minimum_required(VERSION 3.10)

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

configure_file(${LIBRARY_NAME}.pc.in ${LIBRARY_NAME}.pc @ONLY)

target_include_directories(${LIBRARY_NAME} PRIVATE include)

include(GNUInstallDirs)

install(TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)