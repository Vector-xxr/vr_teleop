cmake_minimum_required(VERSION 3.22)
project(openarm_vision_grasp_yolo_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

if(NOT TARGET kdl_parser::kdl_parser)
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()

add_executable(kdl_fk_ik_tool cpp/kdl_fk_ik_tool.cpp)
target_link_libraries(
  kdl_fk_ik_tool
  PRIVATE
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    urdfdom::urdfdom_model
)
