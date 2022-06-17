cmake_minimum_required(VERSION 3.22)
project(hw4)

set(CMAKE_CXX_STANDARD 11)

add_executable(hw4 main.c sim_api.c core_api.cpp)
