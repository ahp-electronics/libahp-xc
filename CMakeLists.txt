cmake_minimum_required(VERSION 2.6)

project(ahp_xc C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

find_package(M REQUIRED)
find_package(Threads REQUIRED)
find_package(URJTAG REQUIRED)
find_package(USB REQUIRED)
find_package(USB1 REQUIRED)
find_package(DFU REQUIRED)
find_package(FTDI REQUIRED)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(AHP_XC_VERSION_MAJOR 1)
set(AHP_XC_VERSION_MINOR 3)
set(AHP_XC_VERSION_RELEASE 3)
set(AHPXC_VERSION "${AHP_XC_VERSION_MAJOR}.${AHP_XC_VERSION_MINOR}.${AHP_XC_VERSION_RELEASE}")
set(AHP_XC_VERSION 0x${AHP_XC_VERSION_MAJOR}${AHP_XC_VERSION_MINOR}${AHP_XC_VERSION_RELEASE})
set(AHPXC_SOVERSION ${AHP_XC_VERSION_MAJOR})

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ahp_xc.c
   )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ahp_xc.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/ahp_xc.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.cmake ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_INCLUDE_PATH})
include_directories(${URJTAG_INCLUDE_DIR})
include_directories(${USB_INCLUDE_DIR})
include_directories(${USB1_INCLUDE_DIR})
include_directories(${DFU_INCLUDE_DIR})
link_directories(${CMAKE_LIBRARY_PATH})

execute_process (COMMAND doxygen ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_library(ahp_xc SHARED ${SOURCES} ${PLATFORM_SOURCES})

if(WIN32)
    target_link_libraries(ahp_xc ws2_32 setupapi cfgmgr32)
else(WIN32)
   set_target_properties(ahp_xc PROPERTIES VERSION ${AHPXC_VERSION} SOVERSION ${AHPXC_SOVERSION})
endif(WIN32)

target_link_libraries(ahp_xc ${URJTAG_LIBRARIES} ${DFU_LIBRARIES} ${USB_LIBRARY} ${FTDI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${M_LIB})

install(TARGETS ahp_xc LIBRARY DESTINATION ${LIB_INSTALL_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ahp_xc.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ahp)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/FindAHPXC.cmake DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules")
