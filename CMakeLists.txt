cmake_minimum_required(VERSION 3.10)
project(RealSenseDemo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RealSense paths
set(REALSENSE_INCLUDE_DIR "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")
set(REALSENSE_LIBRARY_DIR "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64")

# OpenCV
set(OpenCV_DIR "D:/GIT_LIBRARY/Opencv/opencv/build")
find_package(OpenCV REQUIRED)

add_executable(main
    src/main.cpp
    include/Utils.cpp # Make sure Utils.cpp is in src/, not include/
)

target_include_directories(main PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${REALSENSE_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(main
    "${REALSENSE_LIBRARY_DIR}/realsense2.lib"
    ${OpenCV_LIBS}
)

if(WIN32)
    add_custom_command(TARGET main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64/realsense2.dll"
        $<TARGET_FILE_DIR:main>
    )
endif()

