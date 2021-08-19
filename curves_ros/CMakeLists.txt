# Project configuration
cmake_minimum_required (VERSION 3.1.3)
project(curves_ros)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  	message(STATUS "Setting build type to 'Release' as none was specified.")
  	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
	"Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  curves
  kindr_ros
  trajectory_msgs
)

find_package(Eigen3)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    curves
    kindr_ros
    trajectory_msgs
)

add_library(
  ${PROJECT_NAME} 
  src/RosJointTrajectoryInterface.cpp
  src/RosMultiDOFJointTrajectoryInterface.cpp
  src/RosMultiDOFJointTrajectoryTranslationInterface.cpp
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
	include
)

# Header directories listed after 'SYSTEM' are marked as external, and don't generate warnings.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
	find_package(catkin REQUIRED
		COMPONENTS
		${CATKIN_PACKAGE_DEPENDENCIES}
		rostest
	)

	# Run a ROS test (add rostest test-dependency).
	# The *.test file has to be on the top of the list.
	add_rostest_gtest(test_${PROJECT_NAME}
		test/ros_interface.test
		test/TestCurvesRos.cpp
	)
endif()

if(TARGET test_${PROJECT_NAME})
	target_link_libraries(test_${PROJECT_NAME}
		gtest_main
		${PROJECT_NAME}
		${catkin_LIBRARIES}
	)

	target_include_directories(test_${PROJECT_NAME} PRIVATE
		include
	)

	target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
		${catkin_INCLUDE_DIRS}
	)

	# Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
	find_package(cmake_code_coverage QUIET)
	if(cmake_code_coverage_FOUND)
		add_rostest_coverage(TEST_BUILD_TARGETS
			test_${PROJECT_NAME}
		)
	endif(cmake_code_coverage_FOUND)
endif()
