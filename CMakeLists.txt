cmake_minimum_required(VERSION 3.10)
project(mapping)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(main
    Astarvisualize.cpp
)

target_link_libraries(main
    ${OpenCV_LIBS}
)
