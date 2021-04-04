cmake_minimum_required(VERSION 3.0.2)
project(libreflexxestype2)

# Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
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
 
  add_custom_target(
    build_reflexxes ALL
    COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    # copy libs, set-up soname chain
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
    COMMAND ${CMAKE_COMMAND} -E create_symlink libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libreflexxes.so
  )

  # Install?
  if(EXISTS "${CMAKE_INSTALL_PREFIX}/lib/")
    ADD_CUSTOM_COMMAND(TARGET build_reflexxes POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so ${CMAKE_INSTALL_PREFIX}/lib/libreflexxes.so
    )
  endif()

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

endif()
