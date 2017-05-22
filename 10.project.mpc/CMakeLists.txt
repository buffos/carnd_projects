project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-6")
set(CMAKE_C_COMPILER "/usr/bin/gcc-6")


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")


set(sources src/MPC.cpp src/main.cpp)

include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
include_directories(/usr/include)
link_directories(/usr/lib)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)


endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)

