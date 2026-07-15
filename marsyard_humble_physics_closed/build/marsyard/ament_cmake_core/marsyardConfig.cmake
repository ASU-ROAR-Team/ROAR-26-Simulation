# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_marsyard_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED marsyard_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(marsyard_FOUND FALSE)
  elseif(NOT marsyard_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(marsyard_FOUND FALSE)
  endif()
  return()
endif()
set(_marsyard_CONFIG_INCLUDED TRUE)

# output package information
if(NOT marsyard_FIND_QUIETLY)
  message(STATUS "Found marsyard: 2.0.0 (${marsyard_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'marsyard' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${marsyard_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(marsyard_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${marsyard_DIR}/${_extra}")
endforeach()
