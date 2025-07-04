# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_example_behaviors_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED example_behaviors_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(example_behaviors_FOUND FALSE)
  elseif(NOT example_behaviors_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(example_behaviors_FOUND FALSE)
  endif()
  return()
endif()
set(_example_behaviors_CONFIG_INCLUDED TRUE)

# output package information
if(NOT example_behaviors_FIND_QUIETLY)
  message(STATUS "Found example_behaviors: 6.5.0 (${example_behaviors_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'example_behaviors' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT example_behaviors_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(example_behaviors_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${example_behaviors_DIR}/${_extra}")
endforeach()
