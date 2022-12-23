project(OctreeOCL)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

message(${CMAKE_SOURCE_DIR})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL REQUIRED)
find_package(OpenCL REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
  ${OPENCL_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB octree_ocl_SRC
  "src/OCLManager.hpp"
  "src/Octree.hpp"
  "src/OctreeOCL.hpp"
  "src/OctreeOCL.cl"
  "src/HVOctree.cl"
  "src/HVOctree.hpp"
)
#add_library(octree_ocl SHARED ${octree_ocl_SRC})
#target_link_libraries(
#  octree_ocl
#  ${PCL_LIBRARIES}
#  ${OPENCL_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#)
#set_target_properties(octree_ocl PROPERTIES LINKER_LANGUAGE CXX)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(example "src/example.cpp" ${octree_ocl_SRC})
target_link_libraries(
  example
  ${PCL_LIBRARIES}
  ${OPENCL_LIBRARIES}
)
