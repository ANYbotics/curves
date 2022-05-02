cmake_minimum_required(VERSION 3.5.1)
project(curves)

set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

# Glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog libglog REQUIRED)

# Add Doxygen documentation
add_subdirectory(doc/doxygen)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
    Boost
    glog
    kindr
)

add_library(${PROJECT_NAME}
  src/KeyGenerator.cpp
  src/CubicHermiteSE3Curve.cpp
  src/CubicHermiteE3Curve.cpp
  src/SlerpSE3Curve.cpp
  src/polynomial_splines_traits.cpp
  src/helpers.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Header directories listed after 'SYSTEM' are marked as external, and don't generate warnings.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog
)

install(TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/CubicHermiteSE3CurveTest.cpp
    test/PolynomialSplineContainerTest.cpp
    test/PolynomialSplineVectorSpaceCurveTest.cpp
    test/PolynomialSplineQuinticScalarCurveTest.cpp
    test/PolynomialSplinesTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )

  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    glog
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
