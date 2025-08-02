
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")
include_directories( 
    include/plan_env
)

add_library( 
        local_map STATIC
        src/sdf_map.cpp 
    )

