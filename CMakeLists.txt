cmake_minimum_required(VERSION 3.10.2)

project(simple_mobile_robot)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(map)
add_subdirectory(miscellaneous)
add_subdirectory(navigation)
add_subdirectory(robot)

target_include_directories(${PROJECT_NAME} PUBLIC map miscellaneous navigation robot)

target_link_libraries(${PROJECT_NAME} map miscellaneous navigation robot)