cmake_minimum_required(VERSION 3.5.1)
project(robile_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rviz kelo_tulip)

find_package(gazebo)
find_package(roslaunch)

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/config/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(robile_gazebo_platform_controller
    src/KeloDrive.cpp
    src/RobilePlatformController.cpp
    src/main.cpp
)

target_link_libraries(robile_gazebo_platform_controller
    ${catkin_LIBRARIES}
)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
