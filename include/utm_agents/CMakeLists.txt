cmake_minimum_required(VERSION 2.8)
project(AbstractUTMSimulation)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set( LIBS_TO_LINK yaml-cpp )

add_executable(testNNLoad testNNLoad.cpp)
target_link_libraries( testNNLoad ${LIBS_TO_LINK} )
