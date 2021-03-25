cmake_minimum_required(VERSION 3.5)
project(libreflexxestype2)

find_package(ament_cmake REQUIRED)

include_directories(include)

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
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so
    COMMAND make -C "${CMAKE_CURRENT_SOURCE_DIR}/Linux" clean64 all64 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}  
  )

  # Export Reflexxes library for ROS
  add_library(reflexxestype2 ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so)
  target_include_directories(reflexxestype2 PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  set_target_properties(reflexxestype2 PROPERTIES LINKER_LANGUAGE CXX)

  # Install header files
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
  # TODO(andyz): This extra installation step shouldn't be necessary. Look into deleting it.
  # (destogl): This is not possible, except for adding "libreflexxestype2/" to all reflexxes includes
  install(DIRECTORY include/ DESTINATION include/)

  install(
    TARGETS reflexxestype2
    EXPORT export_reflexxestype2
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  # Install library file
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Linux/x64/release/lib/shared/libReflexxesTypeII.so
    DESTINATION lib RENAME libreflexxestype2.so
  )

  ament_export_include_directories(include)
  ament_export_targets(export_reflexxestype2 HAS_LIBRARY_TARGET)
  ament_export_libraries(reflexxestype2)

endif()

ament_package()
