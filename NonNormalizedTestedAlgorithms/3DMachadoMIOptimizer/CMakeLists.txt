
cmake_minimum_required(VERSION 2.8)
 
project(MachadoMIOptimizer3D)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(MachadoMIOptimizer3D MACOSX_BUNDLE MachadoMIOptimizer3D.cxx)
target_link_libraries(MachadoMIOptimizer3D ${ITK_LIBRARIES})
