cmake_minimum_required(VERSION 2.8)

project(SLAM)


#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")



find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/usr/local/eigen3/)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS} ${PCL_LIBRARIES})


