cmake_minimum_required(VERSION 3.10)
project(DronePlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(/usr/include/eigen3)
include_directories(include)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(gz-transport13 REQUIRED)  # ADD THIS LINE

add_executable(uav_planner 
    src/main.cpp 
    src/path_planner.cpp 
    src/world_model.cpp 
    src/trajectory_follower.cpp
      src/lidar_bridge.cpp 
)

target_link_libraries(uav_planner PRIVATE 
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    MAVSDK::mavsdk
    gz-transport13::gz-transport13  # ADD THIS LINE
)
