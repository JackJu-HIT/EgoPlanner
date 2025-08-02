cmake_minimum_required(VERSION 3.0.2)
project(EgoPlanner) 
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(./planner/bspline_opt/include)
include_directories(./planner/path_searching/include)
include_directories(./planner/plan_env/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)
include_directories(./planner/include)

add_subdirectory(planner/plan_manage)
add_subdirectory(planner/bspline_opt)
add_subdirectory(planner/path_searching)
add_subdirectory(planner/plan_env)


add_executable(EgoPlanCore main.cpp)
target_link_libraries( EgoPlanCore 
                       bspline_opt
                       path_searching
                       local_map
                       python2.7 
                       ego_planner

)
