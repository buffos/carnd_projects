file(GLOB sources "*.cpp")
file(GLOB headers "*.h[p]*")
set(sources ${sources})
# remove main from sources to create a library so I can link to both main and test.
list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	
	set(vcpkg_dir "C:/Users/buffo/Code/cpp/vcpkg")
	set(CMAKE_TOOLCHAIN_FILE "${vcpkg_dir}./scripts/buildsystems/vcpkg.cmake")
	
	include_directories("${vcpkg_dir}./installed/x86-windows/include")
	link_directories("${vcpkg_dir}./installed/x86-windows/lib")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

	include_directories(/usr/local/include)
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/lib)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_library(planningLib ${sources} ${headers})
add_executable(path_planning main.cpp)

# linking (linux)
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(path_planning planningLib z ssl uv uWS)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")

# linking (windows)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(path_planning planningLib zlib ssleay32 libuv uWS)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")