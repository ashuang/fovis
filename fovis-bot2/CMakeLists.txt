cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME fovis-bot2)
include(cmake/pods.cmake)

# require pkg-config to build.  We use pkg-config to search for both
# required and optional dependencies (Eigen, LCM, libbot2, etc.)
find_package(PkgConfig REQUIRED)

# check for required dependencies
pkg_check_modules(FOVIS REQUIRED libfovis)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(BOT2_CORE REQUIRED bot2-core)
pkg_check_modules(BOT2_LCMGL REQUIRED bot2-lcmgl-client)

macro(set_found_message name)
    if(${name}_FOUND)
        set(${name}_MESSAGE "Found")
    else()
        set(${name}_MESSAGE "Required, but not found")
    endif()
endmacro()

set_found_message(LCM)
set_found_message(FOVIS)
set_found_message(BOT2_CORE)
set_found_message(BOT2_LCMGL)


# check for optional dependencies
pkg_check_modules(LCMTYPES_KINECT lcmtypes_kinect)


# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

if(LCMTYPES_KINECT_FOUND)
    add_subdirectory(kinect-odometry)
else()
    message("lcmtypes_kinect not found.  Not building kinect-odometry")
endif()

add_subdirectory(stereo-odometry)

message("

Configuration (fovis-bot2):

Dependencies:
       fovis - ${FOVIS_MESSAGE}
         LCM - ${LCM_MESSAGE}
   bot2-core - ${BOT2_CORE_MESSAGE}
  bot2-lcmgl - ${BOT2_LCMGL_MESSAGE}

")
