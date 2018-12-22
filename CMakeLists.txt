cmake_minimum_required(VERSION 2.8)
project(StereoVO)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()
# Ceres
find_package(Ceres REQUIRED)
#mynteye
find_package(mynteye REQUIRED)
set(__link_libs mynteye)
#find_package(Pangolin REQUIRED))
#
## OpenCV

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
#        ${Pangolin_INCLUDE_DIRS}
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
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
        ${CERES_LIBRARIES}
#        ${Pangolin_LIBRARIES}
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(KITTI ./demo/KITTI.cc)
target_link_libraries(KITTI ${PROJECT_NAME})

add_executable(MINT demo/mynteye.cc)
target_link_libraries(MINT ${PROJECT_NAME})