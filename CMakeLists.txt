cmake_minimum_required(VERSION 3.20)
project(pcl_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

set(OpenCV_DIR "D:/opencv/opencv/build/")
find_package(OpenCV REQUIRED)

add_executable(pcl_demo src/main.cpp src/utils.cpp src/ore_analysis.cpp)
target_link_libraries(pcl_demo PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(pcl_demo PRIVATE yaml-cpp)
target_include_directories(pcl_demo PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
target_compile_definitions(pcl_demo PRIVATE ${PCL_DEFINITIONS})

# add_executable (pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
# target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})
# target_include_directories(pcl_visualizer_demo PRIVATE ${PCL_INCLUDE_DIRS} include)
# target_compile_definitions(pcl_visualizer_demo PRIVATE ${PCL_DEFINITIONS})

if (MSVC)
    add_compile_options(/utf-8)
endif()