cmake_minimum_required(VERSION 3.10)
project(RayTracing)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("C:\\Program Files (x86)\\Eigen3\\include")

add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
target_link_libraries(RayTracing ${OpenCV_LIBRARIES})
