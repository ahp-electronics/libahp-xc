# - Try to find AHPXC
# Once done this will define
#
#  AHPXC_FOUND - system has AHPXC
#  AHPXC_INCLUDE_DIR - the AHPXC include directory
#  AHPXC_LIBRARIES - Link these to use AHPXC
#  AHPXC_VERSION_STRING - Human readable version number of ahp_xc
#  AHPXC_VERSION_MAJOR  - Major version number of ahp_xc
#  AHPXC_VERSION_MINOR  - Minor version number of ahp_xc

# Copyright (c) 2017, Ilia Platone, <info@iliaplatone.com>
# Based on FindLibfacile by Carsten Niehaus, <cniehaus@gmx.de>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)

  # in cache already
  set(AHPXC_FOUND TRUE)
  message(STATUS "Found AHPXC: ${AHPXC_LIBRARIES}")


else (AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)

    find_path(AHPXC_INCLUDE_DIR ahp_xc.h
      PATH_SUFFIXES ahp
      ${_obIncDir}
      ${GNUWIN32_DIR}/include
    )

  find_library(AHPXC_LIBRARIES NAMES ahp_xc
    PATHS
    ${_obLinkDir}
    ${GNUWIN32_DIR}/lib
    /usr/local/lib
    HINTS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
  )

if(AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)
  set(AHPXC_FOUND TRUE)
else (AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)
  set(AHPXC_FOUND FALSE)
endif(AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)

  if (AHPXC_FOUND)
    if (NOT AHPXC_FIND_QUIETLY)
      message(STATUS "Found AHPXC: ${AHPXC_LIBRARIES}")
    endif (NOT AHPXC_FIND_QUIETLY)
  else (AHPXC_FOUND)
    if (AHPXC_FIND_REQUIRED)
      message(FATAL_ERROR "AHPXC not found. Please install libahp_xc-dev")
    endif (AHPXC_FIND_REQUIRED)
  endif (AHPXC_FOUND)

  mark_as_advanced(AHPXC_LIBRARIES)
  
endif (AHPXC_INCLUDE_DIR AND AHPXC_LIBRARIES)
