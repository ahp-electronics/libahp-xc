cmake_minimum_required(VERSION 2.6)

project(ahp_xc C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
include(GNUInstallDirs)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(AHPXC_VERSION "1.0.14")
set(AHPXC_SOVERSION "1")
set(SOURCES
    ${CMAKE_SOURCE_DIR}/rs232.c
    ${CMAKE_SOURCE_DIR}/ahp_xc.c
   )

include_directories(${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_LIBRARY_PATH})

add_library(ahp_xc SHARED ${SOURCES})
set_target_properties(ahp_xc PROPERTIES VERSION ${AHPXC_VERSION} SOVERSION ${AHPXC_SOVERSION})

install(TARGETS ahp_xc LIBRARY DESTINATION ${LIB_INSTALL_DIR})
install(FILES ahp_xc.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ahp)
