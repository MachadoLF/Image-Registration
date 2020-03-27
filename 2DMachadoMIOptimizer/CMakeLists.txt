
cmake_minimum_required(VERSION 2.8)
 
project(MachadoMIOptimizer)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(MachadoMIOptimizer MACOSX_BUNDLE MachadoMIOptimizer.cxx)
target_link_libraries(MachadoMIOptimizer ${ITK_LIBRARIES})
