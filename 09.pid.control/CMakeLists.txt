project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")



if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

set(vcpkg_dir "C:/Users/buffo/Code/cpp/vcpkg")
#include_directories(${vcpkg}$/installed/x86-windows/include)
#include_directories(${vcpkg}$/installed/x86-windows/include/openssl)
#include_directories(${vcpkg}$/installed/x86-windows/include/boost)
include_directories("C:/Users/buffo/Code/cpp/vcpkg/installed/x86-windows/include/uWS")
#link_directories(${vcpkg}$/installed/x86-windows/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

set(sources src/PID.cpp src/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(pid ${sources})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_link_libraries(pid z ssl uv uWS)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
