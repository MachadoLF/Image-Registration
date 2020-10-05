
cmake_minimum_required(VERSION 2.8)
 
project(MachadoMIMultiStageDeformable)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(MachadoMIMultiStageDeformable MACOSX_BUNDLE MachadoMIMultiStageDeformable.cxx)
target_link_libraries(MachadoMIMultiStageDeformable ${ITK_LIBRARIES})
