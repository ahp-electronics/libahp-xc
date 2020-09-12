cmake_minimum_required(VERSION 2.6)

project(ahp_xc C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(Threads REQUIRED)
find_package(M REQUIRED)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/rs232.c
    ${CMAKE_SOURCE_DIR}/commands.c
   )
include_directories(${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_LIBRARY_PATH})

add_library(ahp_xc SHARED ${SOURCES})

target_link_libraries(ahp_xc ${M_LIB} ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS ahp_xc LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(FILES ahp_xc.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
