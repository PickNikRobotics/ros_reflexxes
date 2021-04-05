cmake_minimum_required(VERSION 3.0.2)
project(libreflexxestype2)

# Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reflexxes
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
  # Set location of build output
  set(libreflexxes_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so)

  # Build Reflexxes library with a Make file
  add_custom_command(
    OUTPUT ${libreflexxes_LOCATION}
    COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_library(reflexxes ${libreflexxes_LOCATION})
  set_target_properties(reflexxes PROPERTIES LINKER_LANGUAGE CXX)

  # Copy into devel/lib
  add_custom_command(
    TARGET reflexxes POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${libreflexxes_LOCATION} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libreflexxes.so
  )

  # Install header files
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  # Install library file
  install(FILES ${libreflexxes_LOCATION}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RENAME libreflexxes.so
  )
endif()
