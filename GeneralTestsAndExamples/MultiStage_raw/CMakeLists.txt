
cmake_minimum_required(VERSION 2.8)
 
project(MultiStageImageRegistration1)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(MultiStageImageRegistration1 MACOSX_BUNDLE MultiStageImageRegistration1.cxx)
target_link_libraries(MultiStageImageRegistration1 ${ITK_LIBRARIES})
