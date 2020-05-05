
cmake_minimum_required(VERSION 2.8)
 
project(GeneralMachadoMIOptimizer)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(GeneralMachadoMIOptimizer MACOSX_BUNDLE GeneralMachadoMIOptimizer.cxx)
target_link_libraries(GeneralMachadoMIOptimizer ${ITK_LIBRARIES})
