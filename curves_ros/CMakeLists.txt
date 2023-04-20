cmake_minimum_required(VERSION 3.5.1)
project(curves_ros)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
	roscpp
	curves
	kindr_ros
	trajectory_msgs
)

find_package(catkin REQUIRED
	COMPONENTS
		${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
		${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
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
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
	TARGETS
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

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
	find_package(catkin REQUIRED
		COMPONENTS
		${CATKIN_PACKAGE_DEPENDENCIES}
		rostest
	)

	add_rostest_gtest(test_${PROJECT_NAME}
		test/ros_interface.test
		test/TestCurvesRos.cpp
	)

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

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
	add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
