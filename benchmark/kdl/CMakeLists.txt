cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)
add_compile_options(-Ofast)

# benchmark library
find_package(benchmark REQUIRED)
get_target_property(benchmark_INCLUDE_DIRS benchmark::benchmark INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${benchmark_INCLUDE_DIRS})
get_target_property(benchmark_LINK_LIBRARIES benchmark::benchmark INTERFACE_LINK_LIBRARIES)

# FIXME: this info is not in benchmark_... variables?
link_directories(${benchmark_DIR}../../)

# orocos kdl
find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

# pkl
find_package(pkl REQUIRED)
include_directories(${pkl_INCLUDE_DIRS})

set(SOURCES benchmark_main.cpp jacobian.cpp forward.cpp constants.cpp inverse.cpp)
add_executable(pkl_benchmark ${SOURCES})

target_link_libraries(pkl_benchmark ${orocos_kdl_LIBRARIES})
target_link_libraries(pkl_benchmark ${benchmark_LINK_LIBRARIES})
target_link_libraries(pkl_benchmark benchmark pthread)
target_link_libraries(pkl_benchmark ${pkl_LIBRARIES})
