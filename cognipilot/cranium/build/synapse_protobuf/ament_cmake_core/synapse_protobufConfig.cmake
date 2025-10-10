# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_synapse_protobuf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED synapse_protobuf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(synapse_protobuf_FOUND FALSE)
  elseif(NOT synapse_protobuf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(synapse_protobuf_FOUND FALSE)
  endif()
  return()
endif()
set(_synapse_protobuf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT synapse_protobuf_FIND_QUIETLY)
  message(STATUS "Found synapse_protobuf: 0.0.0 (${synapse_protobuf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'synapse_protobuf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${synapse_protobuf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(synapse_protobuf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${synapse_protobuf_DIR}/${_extra}")
endforeach()
