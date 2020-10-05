
cmake_minimum_required(VERSION 2.8)
 
project(NormalizedMachadoMIRunner3D)
 
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
 
add_executable(NormalizedMachadoMIRunner3D MACOSX_BUNDLE NormalizedMachadoMIRunner3D.cxx)
target_link_libraries(NormalizedMachadoMIRunner3D ${ITK_LIBRARIES})
