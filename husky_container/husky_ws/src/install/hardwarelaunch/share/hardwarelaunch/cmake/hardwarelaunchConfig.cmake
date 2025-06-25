# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hardwarelaunch_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hardwarelaunch_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hardwarelaunch_FOUND FALSE)
  elseif(NOT hardwarelaunch_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hardwarelaunch_FOUND FALSE)
  endif()
  return()
endif()
set(_hardwarelaunch_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hardwarelaunch_FIND_QUIETLY)
  message(STATUS "Found hardwarelaunch: 0.0.1 (${hardwarelaunch_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hardwarelaunch' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hardwarelaunch_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hardwarelaunch_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hardwarelaunch_DIR}/${_extra}")
endforeach()
