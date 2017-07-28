file(GLOB test_files "*.cpp")
set(sources ${test_files})

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


add_executable(test_path ${sources})

# linking (linux)
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(test_path planningLib z ssl uv uWS gtest gmock_main)
endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")

# linking (windows)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	target_link_libraries(test_path planningLib zlib ssleay32 libuv uWS gtest gmock_main)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")