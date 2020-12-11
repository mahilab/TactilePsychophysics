#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mahi::daq" for configuration "MinSizeRel"
set_property(TARGET mahi::daq APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(mahi::daq PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/mahi-daq.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS mahi::daq )
list(APPEND _IMPORT_CHECK_FILES_FOR_mahi::daq "${_IMPORT_PREFIX}/lib/mahi-daq.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
