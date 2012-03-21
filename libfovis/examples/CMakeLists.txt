# look for OpenNI

find_path(OPENNI_INCLUDE_DIRS
    NAMES XnOS.h
    PATH_SUFFIXES ni)
if("${OPENNI_INCLUDE_DIRS}" STREQUAL "OPENNI_INCLUDE_DIRS-NOTFOUND")
  set(OPENNI_EXAMPLE_MESSAGE "Disabled - XnOS.h not found" PARENT_SCOPE)
  return()
endif()

find_library(OPENNI_LIBS OpenNI)
if("${OPENNI_LIBS}" STREQUAL "OPENNI_LIBS-NOTFOUND")
  set(OPENNI_EXAMPLE_MESSAGE "Disabled - OpenNI library not found" PARENT_SCOPE)
  return()
endif()
set(OPENNI_EXAMPLE_MESSAGE "Enabled" PARENT_SCOPE)

add_subdirectory(fv-example-openni)
