cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#模式
set( CMAKE_BUILD_TYPE Release )
# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

project(depth_camera_extrinsic_calib)

find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED COMPONENT common io registration visualization features filters)
include_directories(${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECT_NAME} src/depth_camera_extrinsic_calib.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (interactive_icp src/interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (point_to_line src/depth_camera_point_to_line.cpp)
target_link_libraries (point_to_line ${PCL_LIBRARIES} ${OpenCV_LIBS})

