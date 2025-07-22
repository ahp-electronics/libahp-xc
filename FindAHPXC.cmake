# - Try to find AHP_XC
# Once done this will define
#
#  AHP_XC_FOUND - system has AHP_XC
#  AHP_XC_INCLUDE_DIR - the AHP_XC include directory
#  AHP_XC_LIBRARIES - Link these to use AHP_XC
#  AHP_XC_VERSION_STRING - Human readable version number of ahp_xc
#  AHP_XC_VERSION_MAJOR  - Major version number of ahp_xc
#  AHP_XC_VERSION_MINOR  - Minor version number of ahp_xc

# Copyright (c) 2017, Ilia Platone, <info@iliaplatone.com>
# Based on FindLibfacile by Carsten Niehaus, <cniehaus@gmx.de>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES)

  # in cache already
  set(AHP_XC_FOUND TRUE)
  message(STATUS "Found AHP_XC: ${AHP_XC_LIBRARIES}")

  set(AHP_XC_MIN_VERSION_MAJOR @AHP_XC_VERSION_MAJOR@)
  set(AHP_XC_MIN_VERSION_MINOR @AHP_XC_VERSION_MINOR@)
  set(AHP_XC_MIN_VERSION_RELEASE @AHP_XC_VERSION_RELEASE@)

else (AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES)

    find_path(AHP_XC_INCLUDE_DIR ahp_xc.h
      PATH_SUFFIXES ahp
      ${_obIncDir}
      ${GNUWIN32_DIR}/include
    )

  find_library(AHP_XC_LIBRARIES NAMES ahp_xc
    PATHS
    ${_obLinkDir}
    ${GNUWIN32_DIR}/lib
    /usr/local/lib
    HINTS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
  )

set(AHP_XC_VERSION $(grep '\\version ' ${AHP_XC_INCLUDE_DIR}/ahp_xc.h | cut -d ' ' -f 3))
set(AHP_XC_VERSION_MAJOR $(echo ${AHP_XC_VERSION} | cut -d '.' -f 1))
set(AHP_XC_VERSION_MINOR $(echo ${AHP_XC_VERSION} | cut -d '.' -f 2))
set(AHP_XC_VERSION_RELEASE $(echo ${AHP_XC_VERSION} | cut -d '.' -f 3))

if(AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MAJOR}  AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MINOR}  AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_RELEASE})
  set(AHP_XC_FOUND TRUE)
else (AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MAJOR}  AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MINOR}  AND
        AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_RELEASE})
  set(AHP_XC_FOUND FALSE)
endif(AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES AND
    AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MAJOR}  AND
    AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_MINOR}  AND
    AHP_XC_VERSION_MAJOR >= ${AHP_XC_MIN_VERSION_RELEASE})

  if (AHP_XC_FOUND)
    if (NOT AHP_XC_FIND_QUIETLY)
      message(STATUS "Found AHP_XC: ${AHP_XC_LIBRARIES}")
    endif (NOT AHP_XC_FIND_QUIETLY)
  else (AHP_XC_FOUND)
    if (AHP_XC_FIND_REQUIRED)
      message(FATAL_ERROR "AHP_XC not found. Please install libahp_xc-dev")
    endif (AHP_XC_FIND_REQUIRED)
  endif (AHP_XC_FOUND)

  mark_as_advanced(AHP_XC_LIBRARIES)
  
endif (AHP_XC_INCLUDE_DIR AND AHP_XC_LIBRARIES)
