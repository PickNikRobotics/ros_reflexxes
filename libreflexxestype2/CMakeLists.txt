cmake_minimum_required(VERSION 3.5)
project(libreflexxestype2)

find_package(ament_cmake REQUIRED)

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
 
  # Build Reflexxes library
  add_custom_target(
    build_reflexxes ALL
        COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        # Install library
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/static/libReflexxesTypeII.a ${CMAKE_INSTALL_PREFIX}/lib/libReflexxesTypeII.a
  )

  # Install header files
  install(DIRECTORY include/ DESTINATION include)

  ament_export_libraries(export_${PROJECT_NAME})
  ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

endif()
