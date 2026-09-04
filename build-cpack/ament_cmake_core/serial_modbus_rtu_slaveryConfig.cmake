# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_serial_modbus_rtu_slavery_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED serial_modbus_rtu_slavery_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(serial_modbus_rtu_slavery_FOUND FALSE)
  elseif(NOT serial_modbus_rtu_slavery_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(serial_modbus_rtu_slavery_FOUND FALSE)
  endif()
  return()
endif()
set(_serial_modbus_rtu_slavery_CONFIG_INCLUDED TRUE)

# output package information
if(NOT serial_modbus_rtu_slavery_FIND_QUIETLY)
  message(STATUS "Found serial_modbus_rtu_slavery: 1.0.0 (${serial_modbus_rtu_slavery_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'serial_modbus_rtu_slavery' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${serial_modbus_rtu_slavery_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(serial_modbus_rtu_slavery_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${serial_modbus_rtu_slavery_DIR}/${_extra}")
endforeach()
