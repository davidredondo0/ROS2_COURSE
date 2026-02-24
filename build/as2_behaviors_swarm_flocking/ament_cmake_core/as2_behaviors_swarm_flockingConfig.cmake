# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_as2_behaviors_swarm_flocking_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED as2_behaviors_swarm_flocking_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(as2_behaviors_swarm_flocking_FOUND FALSE)
  elseif(NOT as2_behaviors_swarm_flocking_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(as2_behaviors_swarm_flocking_FOUND FALSE)
  endif()
  return()
endif()
set(_as2_behaviors_swarm_flocking_CONFIG_INCLUDED TRUE)

# output package information
if(NOT as2_behaviors_swarm_flocking_FIND_QUIETLY)
  message(STATUS "Found as2_behaviors_swarm_flocking: 1.1.3 (${as2_behaviors_swarm_flocking_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'as2_behaviors_swarm_flocking' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${as2_behaviors_swarm_flocking_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(as2_behaviors_swarm_flocking_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${as2_behaviors_swarm_flocking_DIR}/${_extra}")
endforeach()
