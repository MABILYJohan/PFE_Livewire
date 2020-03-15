cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-icp)

find_package(PCL 1.5 REQUIRED)

#~ list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Crée des variables avec les fichiers à compiler
set(SRCS
    icp.cpp
    utilsPcl.cpp
    )
    
set(HEADERS
    utilsPcl.h
    )

# On indique que l'on veut un exécutable "icp" compilé à partir des fichiers décrits par les variables SRCS et HEADERS
add_executable(icp ${SRCS} ${HEADERS})
#~ add_executable (icp icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})
