# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_a4g16_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED a4g16_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(a4g16_FOUND FALSE)
  elseif(NOT a4g16_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(a4g16_FOUND FALSE)
  endif()
  return()
endif()
set(_a4g16_CONFIG_INCLUDED TRUE)

# output package information
if(NOT a4g16_FIND_QUIETLY)
  message(STATUS "Found a4g16: 0.0.0 (${a4g16_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'a4g16' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${a4g16_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(a4g16_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${a4g16_DIR}/${_extra}")
endforeach()
