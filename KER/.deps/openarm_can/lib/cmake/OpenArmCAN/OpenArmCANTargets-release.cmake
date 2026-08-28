#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenArmCAN::openarm_can" for configuration "Release"
set_property(TARGET OpenArmCAN::openarm_can APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OpenArmCAN::openarm_can PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopenarm_can.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenArmCAN::openarm_can )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenArmCAN::openarm_can "${_IMPORT_PREFIX}/lib/libopenarm_can.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
