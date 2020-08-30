cmake_minimum_required(VERSION 2.8.3)
project(global_optimize)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Ceres REQUIRED)

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_DIRS} )

if (NOT TARGET plugin_)
    add_subdirectory(./ThirdParty/GeographicLib/)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)


catkin_package()


add_executable(global_optimize_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_optimize_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} libGeographicc) 
