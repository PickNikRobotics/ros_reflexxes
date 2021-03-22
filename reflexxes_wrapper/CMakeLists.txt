cmake_minimum_required(VERSION 3.5)
project(reflexxes_wrapper)

find_package(catkin REQUIRED COMPONENTS
  libreflexxestype2
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

#ament_export_include_directories(include)
