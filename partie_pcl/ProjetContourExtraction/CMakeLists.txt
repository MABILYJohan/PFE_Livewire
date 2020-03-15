cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ProjetContourExtraction)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Crée des variables avec les fichiers à compiler
set(SRCS
	main.cpp
    extractContour.cpp
    utilsPcl.cpp
    )
    
set(HEADERS
    extractContour.h
    utilsPcl.h
    )

# On indique que l'on veut un exécutable "icp" compilé à partir des fichiers décrits par les variables SRCS et HEADERS
add_executable(extractContour ${SRCS} ${HEADERS})

#~ add_executable (extractContour extractContour.cpp)
target_link_libraries (extractContour ${PCL_LIBRARIES})
