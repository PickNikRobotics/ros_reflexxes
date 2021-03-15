cmake_minimum_required(VERSION 3.5)
project(libreflexxestype2)

find_package(ament_cmake REQUIRED)

include_directories(
  include
)

find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin) 
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "+++++++++++++++++ DETECTED 64 bit +++++++++++++++++")
    set(SUPPORTED "true")
#   elseif(CMAKE_DPKG_ARCH MATCHES "i386")
#     message(STATUS "+++++++++++++++++ DETECTED 32 bit +++++++++++++++++")
#     set(SUPPORTED "true")
  else()
    message(WARNING "+++++++++++++++++ UNSUPPORTED ARCHITECTURE, UNABLE TO USE ReflexxesTypeII +++++++++++++++++")
    hset(SUPPORTED "false")
  endif()
endif()

if(SUPPORTED MATCHES "true")
 
  # Build Reflexxes library with a Make file
  add_custom_target(
    reflexxes ALL
        COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Export Reflexxes library for ROS
  add_library(reflexxestype2 ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/static/libReflexxesTypeII.a)
  ament_target_dependencies(reflexxestype2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
  set_target_properties(reflexxestype2 PROPERTIES LINKER_LANGUAGE CXX)

  ament_export_libraries(reflexxestype2)
  ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

  # Install header files
  install(DIRECTORY include/ DESTINATION include/libreflexxestype2)


  install(
    TARGETS reflexxestype2
    EXPORT export_ftd2xx
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

endif()

ament_package()
