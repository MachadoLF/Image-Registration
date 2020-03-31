
cmake_minimum_required(VERSION 2.8)
 
project(NormalizedMachadoMIOptimizer)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(NormalizedMachadoMIOptimizer MACOSX_BUNDLE NormalizedMachadoMIOptimizer.cxx)
target_link_libraries(NormalizedMachadoMIOptimizer ${ITK_LIBRARIES})
