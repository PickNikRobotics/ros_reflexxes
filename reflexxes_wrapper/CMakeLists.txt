cmake_minimum_required(VERSION 3.5)
project(reflexxes_wrapper)

find_package(ament_cmake REQUIRED)

include_directories(include)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)

ament_package()
