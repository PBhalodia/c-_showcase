cmake_minimum_required(VERSION 3.10.2)

project(simple_mobile_robot)

add_executable(${PROJECT_NAME} src/main.cpp)

add_subdirectory(src/map)
add_subdirectory(src/miscellaneous)
add_subdirectory(src/navigation)
add_subdirectory(src/robot)

target_include_directories(${PROJECT_NAME} PUBLIC
    src/map
    src/miscellaneous
    src/navigation
    src/robot
)

target_link_libraries(${PROJECT_NAME} map miscellaneous navigation robot)