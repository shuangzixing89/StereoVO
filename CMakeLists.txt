cmake_minimum_required(VERSION 3.12)
project(StereoVO)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Ceres
find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

include_directories(
        ./include
)
add_library(${PROJECT_NAME} SHARED
        ./src/frame.cc
        ./src/track.cc
        ./src/map.cc
        ./src/mappoint.cc
        ./src/config.cc
        ./src/camera.cc
        )
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        )

add_executable(Stereo_VO main.cpp)
target_link_libraries(Stereo_VO ${PROJECT_NAME})