cmake_minimum_required(VERSION 3.10)
project(ArmorDetector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include)

add_executable(armor_detector
    src/armor_detector.cpp
    src/armor_detector_main.cpp
)

target_link_libraries(armor_detector ${OpenCV_LIBS})