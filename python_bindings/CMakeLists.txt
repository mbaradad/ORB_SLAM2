#TODO: this compiles/works with python3, but should be beautified
cmake_minimum_required(VERSION 2.8)
project(orb_slam2)

set(ORB_SOURCE_DIR "..")

add_subdirectory(pybind11)
pybind11_add_module(orb_slam2
        src/orb_slam_bindings.cpp
        src/ndarray_converter.cpp)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

message(STATUS "WAAAAAAARNING! The DBoW and Thirdpart libraries must be recompiled on the folders and ../lib/libOrbSLAM.so must be moved to ./lib/ after compiling")


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

#find_package(Eigen3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        "src"
        "${ORB_SOURCE_DIR}/"
        "${ORB_SOURCE_DIR}/include"
        ${EIGEN3_INCLUDE_DIR}
)

#The private tag is important (and not as obvious from pybind11 doc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${PROJECT_SOURCE_DIR}/lib/libORB_SLAM.so
)
