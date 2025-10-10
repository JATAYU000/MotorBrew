#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "synapse_protobuf::synapse_protobuf" for configuration ""
set_property(TARGET synapse_protobuf::synapse_protobuf APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(synapse_protobuf::synapse_protobuf PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsynapse_protobuf.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS synapse_protobuf::synapse_protobuf )
list(APPEND _IMPORT_CHECK_FILES_FOR_synapse_protobuf::synapse_protobuf "${_IMPORT_PREFIX}/lib/libsynapse_protobuf.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
