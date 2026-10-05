# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_waypoint_generator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED waypoint_generator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(waypoint_generator_FOUND FALSE)
  elseif(NOT waypoint_generator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(waypoint_generator_FOUND FALSE)
  endif()
  return()
endif()
set(_waypoint_generator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT waypoint_generator_FIND_QUIETLY)
  message(STATUS "Found waypoint_generator: 0.1.0 (${waypoint_generator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'waypoint_generator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${waypoint_generator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(waypoint_generator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${waypoint_generator_DIR}/${_extra}")
endforeach()
