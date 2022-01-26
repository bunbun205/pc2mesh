cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(TestProject)

find_package(PCL 1.12 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(Test main.cpp)
target_link_libraries(Test ${PCL_LIBRARIES} liblz4.so libpthread.so)