pkg_check_modules(BOT_CORE bot2-core)
if(NOT BOT_CORE_FOUND)
    message("bot2-core not found.  Not building kinect-odometry")
    return()
endif()

add_executable(kinect-odometry
    jpeg-utils.c
    main.cpp
    )

pods_use_pkg_config_packages(kinect-odometry
    glib-2.0
    lcmtypes_kinect
    lcm
    libfovis
    bot2-core)

# link against the LCM types defined in ../lcmtypes/
target_link_libraries(kinect-odometry ${LCMTYPES_LIBS})

# use LCMGL if available
pkg_check_modules(LCMGL bot2-lcmgl-client)
if(LCMGL_FOUND)
    target_link_libraries(kinect-odometry ${LCMGL_LIBRARIES})
    include_directories(${LCMGL_INCLUDE_DIRS})
    add_definitions(-DUSE_LCMGL)
endif()

pods_install_executables(kinect-odometry)

target_link_libraries(kinect-odometry
    z
    jpeg
    )
