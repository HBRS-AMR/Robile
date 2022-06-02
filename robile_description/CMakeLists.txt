cmake_minimum_required(VERSION 3.5.1)
project(robile_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY meshes robots urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
