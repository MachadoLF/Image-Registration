
cmake_minimum_required(VERSION 2.8)
 
project(NormalizedMachadoMI2DRunner)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(NormalizedMachadoMI2DRunner MACOSX_BUNDLE NormalizedMachadoMI2DRunner.cxx)
target_link_libraries(NormalizedMachadoMI2DRunner ${ITK_LIBRARIES})
