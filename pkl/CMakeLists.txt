cmake_minimum_required(VERSION 2.8)

project(pkl)

add_compile_options(-Ofast)
add_compile_options(-fPIC)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
include(${PROJECT_SOURCE_DIR}/../config/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
endif()
include_directories(${Eigen_INCLUDE_DIR})

set(HEADERS
    include/pkl/chain.hpp
    include/pkl/chain.inl
    include/pkl/joint.hpp
    include/pkl/typedef.h
    include/pkl/dual_quaternion.hpp
    include/pkl/homogeneous.hpp
    include/pkl/matrix_rp.hpp
    include/pkl/joint_default_impl.hpp)
set(SOURCES
    src/chain.cpp
    src/joint_dual_quaternion.cpp
    src/joint_homogeneous.cpp
    src/joint_matrix_rp.cpp)
add_library(pkl STATIC ${SOURCES})

# in case we need external libraries
# target_link_libraries(pkl ${orocos_kdl_LIBRARIES})

export(PACKAGE pkl)

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(config/pklConfig.cmake.in
  "${PROJECT_BINARY_DIR}/pklConfig.cmake" @ONLY)
# ... for the install tree
configure_file(config/pklConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/pklConfig.cmake" @ONLY)
# ... for both
configure_file(config/pklConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/pklConfigVersion.cmake" @ONLY)

# Install the pklConfig.cmake and pklConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/pklConfig.cmake"
  "${PROJECT_BINARY_DIR}/pklConfigVersion.cmake"
  DESTINATION share/pkl/cmake COMPONENT dev)

install(FILES ${HEADERS} DESTINATION include/pkl)
install(TARGETS pkl DESTINATION bin EXPORT pklTargets)
install(EXPORT pklTargets DESTINATION share/pkl/cmake/)
