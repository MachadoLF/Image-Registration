
cmake_minimum_required(VERSION 2.8)
 
project(MachadoMIRunner)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
if (ITKVtkGlue_LOADED)
  find_package(VTK REQUIRED)
  include(${VTK_USE_FILE})
endif()
 
add_executable(MachadoMIRunner MACOSX_BUNDLE MachadoMIRunner.cxx)
target_link_libraries(MachadoMIRunner ${ITK_LIBRARIES})
