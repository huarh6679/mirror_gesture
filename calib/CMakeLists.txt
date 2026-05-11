cmake_minimum_required(VERSION 3.10)
project(StereoCameraCalibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.0")
    message(FATAL_ERROR "OpenCV 3.0+ is required")
endif()

find_package(YAML-cpp REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCES
    main.cpp
    camera_calib.cpp
)

add_executable(stereo_calib ${SOURCES})

target_link_libraries(stereo_calib
    ${OpenCV_LIBS}
)

add_executable(split_stereo 
    main_split.cpp
    split_stereo.cpp
)
target_link_libraries(split_stereo
    ${OpenCV_LIBS}
    yaml-cpp
)
