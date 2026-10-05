# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_grid_path_searcher_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED grid_path_searcher_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(grid_path_searcher_FOUND FALSE)
  elseif(NOT grid_path_searcher_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(grid_path_searcher_FOUND FALSE)
  endif()
  return()
endif()
set(_grid_path_searcher_CONFIG_INCLUDED TRUE)

# output package information
if(NOT grid_path_searcher_FIND_QUIETLY)
  message(STATUS "Found grid_path_searcher: 0.1.0 (${grid_path_searcher_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'grid_path_searcher' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${grid_path_searcher_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(grid_path_searcher_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${grid_path_searcher_DIR}/${_extra}")
endforeach()
