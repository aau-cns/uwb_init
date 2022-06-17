cmake_minimum_required(VERSION 3.0.2)
project(uwb_init_cpp)

## Determine CMAKE options
#set(CMAKE_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(UWB_FULL_VERBOSE "ON")

## Try to compile with newer versions of C++
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++20" COMPILER_SUPPORTS_CXX20)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX20)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
elseif(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++0x support. Please use a different C++ compiler.")
endif()

## enforcing cleaner code by adding -Wall and -Werror, the compiler will not ignore warnings anymore
add_definitions(-Wall -Werror)

# Enable compile optimizations

## Set CMAKE compile flags
if (CMAKE_BUILD_TYPE STREQUAL "Release")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif ()


## Display compiler flags
message("Build Type: " ${CMAKE_BUILD_TYPE})
message(STATUS "CMAKE_CXX_FLAGES: ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure message_generation std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs cv_bridge)
find_package(mission_sequencer)
find_package(evb1000_driver)
find_package(mdek_uwb_driver)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

## Display packages version
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    UwbAnchor.msg
    UwbAnchorArrayStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#  DIRECTORY srv
#  FILES Service1.srv Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#  DIRECTORY act
#  FILES Action1.action Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
 cfg/UwbInit.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs nav_msgs sensor_msgs geometry_msgs evb1000_driver mission_sequencer
#  LIBRARIES custom_libraries
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR}
)

## List third party libraries used by all binaries
list(
  APPEND thirdparty_libraries ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES}
)

## List custom libraries inside lib folder
# list(
#   APPEND custom_libraries_sources lib/library_name/src/library_source.cpp
# )

## Declare a C++ library
# add_library(custom_libraries SHARED ${custom_libraries_sources})

## Specify libraries to link a library or executable target against
# target_link_libraries(custom_libraries ${thirdparty_libraries})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(headers_types
    include/types/buffers.hpp
    include/types/buffers/timed_buffer.hpp
    include/types/buffers/data_buffer.hpp
    include/types/randomizer.hpp
    include/types/uwb_anchor.hpp
    include/types/uwb_data.hpp
    include/types/types.hpp
)

set(headers_utils
    include/utils/parse_ros.hpp
    include/utils/logging.hpp
)

set(headers
    ${headers_types}
    ${headers_utils}
    include/options/uwb_init_options.hpp
    include/uwb_wrapper.hpp
    include/uav_init/uwb_init.hpp
    include/uav_init/uwb_init.cpp
)

set(sources
    src/main.cpp
    src/uwb_wrapper.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} ${headers} ${sources})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME uwb_init_cpp PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${thirdparty_libraries})

if(DEFINED UWB_FULL_VERBOSE)
    message("full verbose output is activated")
    set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS FULL_VERBOSE)
else()
    message("full verbose output is deactivated")
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS \${PROJECT_NAME}_node
#   RUNTIME DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS \${PROJECT_NAME}
#   ARCHIVE DESTINATION \${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION \${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION \${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/\${PROJECT_NAME}/
#   DESTINATION \${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(\${PROJECT_NAME}-test test/test_tmp_project.cpp)
# if(TARGET \${PROJECT_NAME}-test)
#   target_link_libraries(\${PROJECT_NAME}-test \${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

###################
## Documentation ##
###################


# Meta information about the project
set(META_PROJECT_NAME        "uwb_init_cpp")
set(META_PROJECT_DESCRIPTION "UWB Intitialization ROS Wrapper")
set(META_AUTHOR_ORGANIZATION "University Klagenfurt: CNS")
set(META_AUTHOR_DOMAIN       "https://gitlab.aau.at/aau-cns/ros_pkgs/uwb_init_cpp")
set(META_AUTHOR_MAINTAINER   "martin.scheiber@ieee.org")
set(META_VERSION_MAJOR       "1")
set(META_VERSION_MINOR       "0")
set(META_VERSION_PATCH       "0")
set(META_VERSION_REVISION    "${GIT_REV}")
set(META_VERSION             "${META_VERSION_MAJOR}.${META_VERSION_MINOR}.${META_VERSION_PATCH}")
set(META_NAME_VERSION        "${META_PROJECT_NAME} v${META_VERSION} (${META_VERSION_REVISION})")
set(META_CMAKE_INIT_SHA      "${GIT_REV}")


option(OPTION_BUILD_DOCS      "Build documentation."                                   ON)
add_subdirectory(docs)
