# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_corti_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED corti_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(corti_FOUND FALSE)
  elseif(NOT corti_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(corti_FOUND FALSE)
  endif()
  return()
endif()
set(_corti_CONFIG_INCLUDED TRUE)

# output package information
if(NOT corti_FIND_QUIETLY)
  message(STATUS "Found corti: 0.0.0 (${corti_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'corti' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${corti_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(corti_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${corti_DIR}/${_extra}")
endforeach()
