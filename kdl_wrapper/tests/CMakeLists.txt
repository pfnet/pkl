cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)
add_compile_options(-Ofast)

# orocos kdl
find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

# kdl parser
find_package(kdl_parser REQUIRED)
include_directories(${kdl_parser_INCLUDE_DIRS})

# gtest
# https://cmake.org/cmake/help/v3.0/module/FindGTest.html
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(kdl_wrapper_test kdl_wrapper_test.cpp)

# eigen
find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
include(../../config/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()
include_directories(${Eigen_INCLUDE_DIR})

# pkl
find_package(pkl REQUIRED)
include_directories(${pkl_INCLUDE_DIRS})

enable_testing()
add_test(kdl_wrapper_test kdl_wrapper_test)

target_link_libraries(kdl_wrapper_test ${GTEST_BOTH_LIBRARIES} ${pkl_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} pthread)
