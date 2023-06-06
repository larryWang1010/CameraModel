cmake_minimum_required(VERSION 3.16)
project(camera_model)

# Build setting
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set dictionary 
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# set complier arguements 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC ${COMPILER_WARNINGS} -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations -Wno-deprecated-declarations")

option(BUILD_ENABLE_DEBUG "print debug info" ON)

if(BUILD_ENABLE_DEBUG)
    add_definitions(-DENABLE_DEBUG)
else()
    remove_definitions(-DENABLE_DEBUG)
endif()

# Debug and log messages
if(CMAKE_BUILD_TYPE MATCHES "Debug")
	add_definitions(-D_DEBUG_MODE_)
endif(CMAKE_BUILD_TYPE MATCHES "Debug")

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# library: OpenCV Eigen glog
find_package(OpenCV 3.4.3 REQUIRED)
find_package(Eigen3 REQUIRED)       
find_package(Glog 0.3.5 REQUIRED)   

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include)

# add librarys
set(LINK_LIBS ${OpenCV_LIBS} ${GLOG_LIBRARY})
set(CAMMODEL_CPP src/abstract_camera.cpp src/MEI_camera.cpp src/pinhole_camera.cpp)

add_library(${PROJECT_NAME} STATIC ${CAMMODEL_CPP})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS}) 

# build test
add_executable(test test/remove_distort.cpp)
target_link_libraries(test ${PROJECT_NAME})