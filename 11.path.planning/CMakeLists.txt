project(Path_Planning)

cmake_minimum_required (VERSION 3.5)


 add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_subdirectory(src)
add_subdirectory(test)

enable_testing ()
add_test (NAME MyTest COMMAND test_path)
