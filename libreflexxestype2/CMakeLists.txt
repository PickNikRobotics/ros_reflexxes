cmake_minimum_required(VERSION 3.0.2)
project(libreflexxestype2)

## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reflexxestype2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin) 
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "+++++++++++++++++ DETECTED 64 bit +++++++++++++++++")
    set(SUPPORTED "true")
  else()
    message(WARNING "+++++++++++++++++ UNSUPPORTED ARCHITECTURE, UNABLE TO USE ReflexxesTypeII +++++++++++++++++")
    hset(SUPPORTED "false")
  endif()
endif()

if(SUPPORTED MATCHES "true")
 
  # Build Reflexxes library with a Make file
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/static/libReflexxesTypeII.a
    COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  # Export Reflexxes library for ROS
  add_library(reflexxestype2 ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/static/libReflexxesTypeII.a)
  set_target_properties(reflexxestype2 PROPERTIES LINKER_LANGUAGE CXX)

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}/
  )

endif()
