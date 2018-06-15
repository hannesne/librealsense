# Install script for directory: C:/Dev/librealsense

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/librealsense2")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Dev/librealsense/Debug/realsense2.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Dev/librealsense/Release/realsense2.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Dev/librealsense/MinSizeRel/realsense2.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Dev/librealsense/RelWithDebInfo/realsense2.lib")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Dev/librealsense/Debug/realsense2.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Dev/librealsense/Release/realsense2.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Dev/librealsense/MinSizeRel/realsense2.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Dev/librealsense/RelWithDebInfo/realsense2.dll")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "C:/Program Files/librealsense2/include/librealsense2/rs.hpp;C:/Program Files/librealsense2/include/librealsense2/rs.h;C:/Program Files/librealsense2/include/librealsense2/rs_context.h;C:/Program Files/librealsense2/include/librealsense2/rs_device.h;C:/Program Files/librealsense2/include/librealsense2/rs_frame.h;C:/Program Files/librealsense2/include/librealsense2/rs_types.h;C:/Program Files/librealsense2/include/librealsense2/rs_sensor.h;C:/Program Files/librealsense2/include/librealsense2/rs_option.h;C:/Program Files/librealsense2/include/librealsense2/rs_processing.h;C:/Program Files/librealsense2/include/librealsense2/rs_record_playback.h;C:/Program Files/librealsense2/include/librealsense2/rs_pipeline.h;C:/Program Files/librealsense2/include/librealsense2/rs_internal.h;C:/Program Files/librealsense2/include/librealsense2/rsutil.h;C:/Program Files/librealsense2/include/librealsense2/rs_advanced_mode.h;C:/Program Files/librealsense2/include/librealsense2/rs_advanced_mode_command.h;C:/Program Files/librealsense2/include/librealsense2/rs_types.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_context.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_device.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_frame.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_processing.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_pipeline.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_record_playback.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_sensor.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_internal.hpp;C:/Program Files/librealsense2/include/librealsense2/rs_advanced_mode.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "C:/Program Files/librealsense2/include/librealsense2" TYPE FILE FILES
    "C:/Dev/librealsense/include/librealsense2/rs.hpp"
    "C:/Dev/librealsense/include/librealsense2/rs.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_context.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_device.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_frame.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_types.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_sensor.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_option.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_processing.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_record_playback.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_pipeline.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_internal.h"
    "C:/Dev/librealsense/include/librealsense2/rsutil.h"
    "C:/Dev/librealsense/include/librealsense2/rs_advanced_mode.h"
    "C:/Dev/librealsense/include/librealsense2/h/rs_advanced_mode_command.h"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_types.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_context.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_device.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_frame.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_processing.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_pipeline.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_record_playback.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_sensor.hpp"
    "C:/Dev/librealsense/include/librealsense2/hpp/rs_internal.hpp"
    "C:/Dev/librealsense/include/librealsense2/rs_advanced_mode.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Dev/librealsense/include/librealsense2")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2/realsense2Targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2/realsense2Targets.cmake"
         "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2/realsense2Targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2/realsense2Targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets-debug.cmake")
  endif()
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets-minsizerel.cmake")
  endif()
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets-relwithdebinfo.cmake")
  endif()
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/CMakeFiles/Export/lib/cmake/realsense2/realsense2Targets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/realsense2Config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/realsense2" TYPE FILE FILES "C:/Dev/librealsense/realsense2ConfigVersion.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND ldconfig)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Dev/librealsense/config/realsense2.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Dev/librealsense/third-party/realsense-file/cmake_install.cmake")
  include("C:/Dev/librealsense/third-party/glfw/cmake_install.cmake")
  include("C:/Dev/librealsense/examples/cmake_install.cmake")
  include("C:/Dev/librealsense/tools/cmake_install.cmake")
  include("C:/Dev/librealsense/wrappers/pcl/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Dev/librealsense/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
