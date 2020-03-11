cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(convertXyzToPcd)

find_package(PCL 1.3 REQUIRED)

#~ list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(convertXyzToPcd convertXyzToPcd.cpp )
target_link_libraries(convertXyzToPcd ${PCL_LIBRARIES})
