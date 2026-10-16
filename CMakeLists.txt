cmake_minimum_required(VERSION 3.16.3)
project(curves)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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



endif()

ament_package()
