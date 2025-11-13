# 1. Set the minimum CMake version
cmake_minimum_required(VERSION 3.10)

# 2. Define our project's name
project(DronePlanner)

# 3. Tell CMake to use the C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 4. Tell CMake where to find our header files
include_directories(include)

# 5. Find our libraries
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(MAVSDK REQUIRED) # <-- NEW: Find MAVSDK
find_package(gz-transport13 REQUIRED)

# 6. Tell CMake to build an executable
add_executable(uav_planner src/main.cpp src/path_planner.cpp src/world_model.cpp src/trajectory_follower.cpp)

# 7. Link our libraries
target_link_libraries(uav_planner PRIVATE 
    Eigen3::Eigen
    ${PCL_LIBRARIES} 

    # --- NEW SIMPLER MAVSDK LINK ---
    # The new MAVSDK links everything with one line
    MAVSDK::mavsdk
    gz-transport13::gz-transport13
)