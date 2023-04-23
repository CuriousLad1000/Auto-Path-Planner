#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Franka::Franka" for configuration "Release"
set_property(TARGET Franka::Franka APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Franka::Franka PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Poco::Foundation;Poco::Net"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfranka.so.0.7.1"
  IMPORTED_SONAME_RELEASE "libfranka.so.0.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS Franka::Franka )
list(APPEND _IMPORT_CHECK_FILES_FOR_Franka::Franka "${_IMPORT_PREFIX}/lib/libfranka.so.0.7.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
