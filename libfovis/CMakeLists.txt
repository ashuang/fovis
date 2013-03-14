cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME libfovis)
include(cmake/pods.cmake)

# require pkg-config to build.  We use pkg-config to search for both
# required and optional dependencies (Eigen, LCM, libbot2, etc.)
find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)

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
