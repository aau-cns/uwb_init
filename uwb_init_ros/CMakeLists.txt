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
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp.git
    GIT_TAG         master
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
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -fno-unsafe-math-optimizations")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -Werror -Wuninitialized -Wmaybe-uninitialized -pedantic -fno-omit-frame-pointer -fsanitize=undefined")
message(STATUS "CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}.")


## List library source files
list(
    APPEND lib_sources
    ../uwb_init_lib/source/uwb_init.cpp
    ../uwb_init_lib/source/solvers/linear/ls_solver.cpp
    ../uwb_init_lib/source/solvers/nonlinear/nls_solver.cpp
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

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf dynamic_reconfigure message_generation std_msgs nav_msgs sensor_msgs geometry_msgs mdek_uwb_driver mission_sequencer)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    UwbAnchor.msg
    UwbAnchorArrayStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs nav_msgs sensor_msgs geometry_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosbag tf std_msgs nav_msgs sensor_msgs geometry_msgs mdek_uwb_driver
  LIBRARIES uwb_init_lib
)

## List custom libraries inside lib folder
list(
  APPEND ros1_lib_sources
  src/uwb_init_ros.cpp
)

## Declare a C++ library
add_library(ros1_lib STATIC ${ros1_lib_sources})
add_dependencies(ros1_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ros1_lib uwb_init_lib)
target_include_directories(ros1_lib PUBLIC ${include_dirs} ${catkin_INCLUDE_DIRS} include)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/uwb_init_ros_node.cpp)
target_link_libraries(${PROJECT_NAME} ros1_lib ${catkin_LIBRARIES})
