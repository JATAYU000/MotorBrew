#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "synapse_tinyframe::synapse_tinyframe" for configuration ""
set_property(TARGET synapse_tinyframe::synapse_tinyframe APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(synapse_tinyframe::synapse_tinyframe PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsynapse_tinyframe.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS synapse_tinyframe::synapse_tinyframe )
list(APPEND _IMPORT_CHECK_FILES_FOR_synapse_tinyframe::synapse_tinyframe "${_IMPORT_PREFIX}/lib/libsynapse_tinyframe.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
