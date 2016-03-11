cmake_minimum_required(VERSION 2.8.11)
project(ocra-kdl CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED cmake_modules)
find_package(OROCOS-RTT REQUIRED)
find_package(OcraRecipes REQUIRED)
find_package(orocos_kdl REQUIRED)

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_use_package(rtt_roscomm)
orocos_use_package(rtt_rosparam)
orocos_use_package(rtt_rosclock)
orocos_use_package(rtt_ros_kdl_tools)

include_directories(
    libs/include
    ${USE_OROCOS_INCLUDE_DIRS}
    ${OcraRecipes_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(ocra-kdl-model libs/src/OcraKDLModel/OcraKDLModel.cpp)
target_link_libraries(ocra-kdl-model
    ${USE_OROCOS_LIBRARIES}
    ${OcraRecipes_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${catkin_LIBRARIES}
)

orocos_component(ocra-kdl-server bin/src/OcraKDLServer/OcraKDLServer.cpp)
set_property(TARGET ocra-kdl-server PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)
target_link_libraries(ocra-kdl-server
    ocra-kdl-model
    ${USE_OROCOS_LIBRARIES}
    ${OcraRecipes_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${catkin_LIBRARIES}
)