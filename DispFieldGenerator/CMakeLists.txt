
cmake_minimum_required(VERSION 2.8)
 
project(DispFieldGenerator)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(DispFieldGenerator MACOSX_BUNDLE DispFieldGenerator.cxx)
target_link_libraries(DispFieldGenerator ${ITK_LIBRARIES})
