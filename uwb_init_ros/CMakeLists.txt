cmake_minimum_required(VERSION 3.13)

project(
    uwb_init_ros
    VERSION 1.0
    DESCRIPTION "Library for automatic, robust, and accurate Ultra-Wide-Band (UWB) anchors initialization"
    LANGUAGES CXX
)

#############
## Library ##
#############

## Include and set up external libraries
include(FetchContent)

# Eigen
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         3.4.0
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
list(APPEND external Eigen)
list(APPEND include_dirs ${EIGEN3_INCLUDE_DIR})
list(APPEND libs Eigen3::Eigen ${EIGEN3_LIBRARIES})

# YamlCPP

if(NOT YAML_CPP_VERSION)
  set( YAML_CPP_VERSION "0.8.0" )
endif()

FetchContent_Declare(
    yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/${YAML_CPP_VERSION}.tar.gz
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
list(APPEND external yaml-cpp)
list(APPEND include_dirs ${YAML_CPP_INCLUDE_DIR})
list(APPEND libs yaml-cpp::yaml-cpp ${YAML_CPP_LIBRARIES})

FetchContent_MakeAvailable(${external})

message(STATUS "Eigen:    " ${EIGEN3_VERSION})
message(STATUS "Yaml-cpp: " ${YAML_CPP})

## Try to compile with C++ 17 (make sure GCC>=7, clang>=5, MSVC>=19.12)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_STANDARD 17)
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no c++17 support. Please use a different C++ compiler.")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Set compiler ${CMAKE_CXX_COMPILER}.")

## Define build type, Release by default
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE}.")

# Set CMAKE compile flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -fno-unsafe-math-optimizations")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=undefined")
message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}.")


## List library source files
list(
    APPEND lib_sources
    ../uwb_init_lib/source/utils/utils.cpp
    ../uwb_init_lib/source/options/BiasType.cpp
    ../uwb_init_lib/source/options/RANSAC_Options.cpp
    ../uwb_init_lib/source/solvers/linear/ls_solver.cpp
    ../uwb_init_lib/source/solvers/nonlinear/nls_solver.cpp
    ../uwb_init_lib/source/uwb_init.cpp
    ../uwb_init_lib/source/planners/wps_gen.cpp
)

## Declare a C++ library
add_library(uwb_init_lib STATIC ${lib_sources})
target_link_libraries(uwb_init_lib ${libs})

## Define includes
list(APPEND include_dirs ../uwb_init_lib/include)
include_directories(${include_dirs})

#########
## ROS ##
#########

set(ros_packages
  roscpp
  rosbag
  tf
  dynamic_reconfigure
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  mdek_uwb_driver
  uwb_msgs
  mission_sequencer
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${ros_packages})

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ros_packages}
  LIBRARIES
)

## List custom libraries inside lib folder
list(
  APPEND ros1_lib_sources
  src/uwb_init_ros.cpp
)

## Declare a C++ library
add_library(ros1_lib STATIC ${ros1_lib_sources})
add_dependencies(ros1_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros1_lib uwb_init_lib)
target_include_directories(ros1_lib PUBLIC ${include_dirs} ${catkin_INCLUDE_DIRS} include)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/uwb_init_ros_node.cpp)
target_link_libraries(${PROJECT_NAME} ros1_lib ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
)
install(FILES
  launch/default.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
