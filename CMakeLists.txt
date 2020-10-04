cmake_minimum_required(VERSION 3.6)

# Name the project
project(CCGHomework)

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")
endif(NOT MSVC)

# Find MeshLib
find_package(MeshLib REQUIRED HINTS "${CMAKE_SOURCE_DIR}/3rdparty")
if (NOT MeshLib_FOUND)
    message(FATAL_ERROR "MeshLib Not Found!")
endif (NOT MeshLib_FOUND)

# Find OpenGL and GLUT
if(MSVC)
  find_package(freeglut REQUIRED HINTS "${CMAKE_SOURCE_DIR}/3rdparty")
  if (NOT freeglut_FOUND)
    message(FATAL_ERROR "freeglut Not Found!")
  endif (NOT freeglut_FOUND)
else(MSVC)
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
endif(MSVC)

# Find Eigen Numeric Lib
find_package(Eigen REQUIRED HINTS "${CMAKE_SOURCE_DIR}/3rdparty")
if (NOT Eigen_FOUND)
    message(FATAL_ERROR "Eigen Not Found!")
endif (NOT Eigen_FOUND)

# Add module
add_subdirectory("hyperbolic")
