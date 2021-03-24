cmake_minimum_required(VERSION 3.5)
project(reflexxes_wrapper)

find_package(catkin REQUIRED COMPONENTS
  libreflexxestype2
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    libreflexxestype2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
