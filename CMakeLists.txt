cmake_minimum_required(VERSION 3.16.3)
project(curves)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if("$ENV{ROS_VERSION}" STREQUAL "1")

set(CATKIN_PACKAGE_DEPENDENCIES
  kindr
  message_logger
  std_utils
)

find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

# Add Doxygen documentation
add_subdirectory(doc/doxygen)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_library(${PROJECT_NAME}
  src/KeyGenerator.cpp
  src/CubicHermiteSE3Curve.cpp
  src/SlerpSE3Curve.cpp
  src/polynomial_splines_traits.cpp
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
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
    test/PolynomialSplineQuinticScalarCurveTest.cpp
    test/PolynomialSplinesTest.cpp
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

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  message_logger
  std_utils
  Eigen3
  kindr
  )

find_package(ament_cmake REQUIRED)

foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

add_subdirectory(doc/doxygen)

add_library(${PROJECT_NAME} SHARED
  src/KeyGenerator.cpp
  src/CubicHermiteSE3Curve.cpp
  src/SlerpSE3Curve.cpp
  src/polynomial_splines_traits.cpp
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen  # Eigen is special ...
  )

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  )

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/CubicHermiteSE3CurveTest.cpp
    test/PolynomialSplineContainerTest.cpp
    test/PolynomialSplineQuinticScalarCurveTest.cpp
    test/PolynomialSplinesTest.cpp
    )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    )

  find_package(cmake_code_coverage QUIET)

  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS test_${PROJECT_NAME})
  endif(cmake_code_coverage_FOUND)

endif()

ament_package()

endif()
