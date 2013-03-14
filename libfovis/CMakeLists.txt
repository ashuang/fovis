cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME libfovis)
include(cmake/pods.cmake)

# Try to use pkg-config to find Eigen3.  If we don't have pkg-config, then
# just assume that Eigen3 is in the include path.
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN REQUIRED eigen3)
endif()

# Do we want to use SSE?
option(USE_SSE "Enables SSE support" ON)
if(USE_SSE)
    set(USE_SSE_MESSAGE "Enabled")
    add_definitions(-DFOVIS_USE_SSE)
else()
    set(USE_SSE_MESSAGE "Disabled")
endif()

# build the main library
add_subdirectory(libfovis)

add_subdirectory(testers)
add_subdirectory(examples)

message("

Configuration (fovis):

           libfovis:    Enabled
        SSE Support:    ${USE_SSE_MESSAGE}
     OpenNI example:    ${OPENNI_EXAMPLE_MESSAGE}
 libfreenct example:    ${LIBFREENECT_EXAMPLE_MESSAGE}

")


if(NOT PKG_CONFIG_FOUND)
    message("Couldn't find pkg-config.  Hoping that eigen3 is in the include path...")
    message("If you get compile errors related to Eigen, try setting CMAKE_CXX_FLAGS")
    message("")
endif()
