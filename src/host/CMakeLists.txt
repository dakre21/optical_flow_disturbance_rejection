cmake_minimum_required(VERSION 3.8)
project(optical_flow)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

option(BUILD_SHARED_LIBS "Build shared libs" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(depthai REQUIRED)

pkg_check_modules(Libcamera REQUIRED libcamera)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS} ${Libcamera_INCLUDE_DIRS} include)
    include_directories(/usr/local/include/depthai)
    include_directories(/usr/local/include/depthai-shared/3rdparty)
    include_directories(/usr/local/lib/cmake/depthai/dependencies/include)

    add_library(camera_module3 SHARED camera_module3.cpp)

    #add_library(luxonis_camera SHARED luxonis_camera.cpp)

    add_executable(optical_flow main.cpp)
    target_link_libraries(optical_flow PRIVATE ${OpenCV_LIBS} ${Libcamera_LIBRARIES} camera_module3 depthai::opencv)

    add_executable(luxonis_debug luxonis_debug.cpp)
    target_link_libraries(luxonis_debug PRIVATE ${OpenCV_LIBS} depthai::opencv)
else()
    message(FATAL_ERROR "OpenCV not found")
endif()
