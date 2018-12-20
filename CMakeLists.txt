cmake_minimum_required(VERSION 3.12)
project(StereoVO)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package( OpenCV 3.1 REQUIRED )
# Ceres
find_package(Ceres REQUIRED)


include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
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
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(KITTI ./demo/KITTI.cc)
target_link_libraries(KITTI ${PROJECT_NAME})