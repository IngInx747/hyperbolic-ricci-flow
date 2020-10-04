# The MeshLib library

MeshLib is a C++ implement of mesh library based on halfedge data structure, it can represent polygon mesh (such as triangle/quadrangle mesh) and handle Euler operation efficiently. 

## System

The code is only tested on Windows, but it should work on Linux and Mac with minor midifications. If there is any problem on the latter two platforms, please let me know.

## Configuration

### Windows

In my situation, I first create a folder `dev` in disk `C:/`, and then I set `C:/dev/` as a system or user `PATH` variable to make sure that `CMake` can find them automatically.
 
After that, unzip `MeshLib` into `C:/dev/` directly.

## Usage

### Configure in CMakeFiles.txt (**Recommend**)

The following is a minimal example to include `MeshLib`
``` cmake
cmake_minimum_required(VERSION 3.6)

# Name the project
project(example)

# Find MeshLib
find_package(MeshLib REQUIRED)
if (NOT MeshLib_FOUND)
    message(FATAL_ERROR "MeshLib Not Found!")
endif (NOT MeshLib_FOUND)

# Include MeshLib core and the 'include' of the project itsself
include_directories("${MeshLib_DIR}/core")

# (Optional) Manually add the sources
set(SOURCES_ONE ${MeshLib_DIR}/core/bmp/RgbImage.cpp)

# Also, the file(GLOB...) allows for wildcard additions
file(GLOB SOURCES_TWO 
    "include/*.h"
    "src/*.cpp")

# Add an executable target called example to be built from 
# the source files.
add_executable(example ${SOURCES_ONE} ${SOURCES_TWO})
```

### Configure in Visual Studio

Manually add `C:/dev/MeshLib/core` into `Additional Include Directories` directory.

If you need to use the RgbImage class, such as reading a bmp image as texture, `C:/dev/MeshLib/core/bmp/RgbImage.cpp` should be added to the source files.