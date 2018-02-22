cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)
add_compile_options(-Ofast)

# https://cmake.org/cmake/help/v3.0/module/FindGTest.html
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(pkl_test utils.cpp forward_test.cpp jacobian_test.cpp inverse_test.cpp user_impl_test.cpp)

find_package(pkl REQUIRED)
include_directories(${pkl_INCLUDE_DIRS})

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
include(../../config/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()
include_directories(${Eigen_INCLUDE_DIR})

enable_testing()
add_test(pkl_test pkl_test)

target_link_libraries(pkl_test ${GTEST_BOTH_LIBRARIES} ${pkl_LIBRARIES} pthread)
