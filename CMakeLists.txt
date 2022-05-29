cmake_minimum_required(VERSION 3.2)
project(lio)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/Flashfusion")
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)


set(OpenCV_DIR "./Flashfusion/third_party_library/opencv-3.3.0/build")
set(Pangolin_DIR "./Flashfusion/third_party_library/Pangolin-master/build")
set(Sophus_DIR "./Flashfusion/third_party_library/Sophus/build")
set(SOPHUS_INCLUDE_DIR "./Flashfusion/third_party_library/Sophus")

set(GCFUSION_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Flashfusion/Shaders" CACHE PATH "Where the shaders live")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  eigen_conversions
  tf_conversions
  tf
  message_generation
  pcl_conversions
  cv_bridge
)
# find_package(Boost REQUIRED)
find_package(PCL REQUIRED QUIET)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(SuiteSparse REQUIRED)

find_package(OpenGL)
find_package(GLUT)
find_package(Chisel REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(PkgConfig)
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
find_package(Boost COMPONENTS system thread  filesystem REQUIRED)
set(realsense2_DIR "./Flashfusion/third_party_library/librealsense-master/build")
set(REALSENSE2_INCLUDE_DIRS "./Flashfusion/third_party_library/librealsense-master/include")
find_library(REALSENSE2_FOUND realsense2 HINTS ${realsense2_DIR} REQUIRED)

if(NOT REALSENSE2_FOUND)
    message(WARN "Failed to find_library(realsense2)")
else()
    message( STATUS "Realsense lib found: " ${REALSENSE2_FOUND})
    message( STATUS "Realsense incl dir: " ${REALSENSE2_INCLUDE_DIRS})
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CameraMeasurement.msg
  FeatureMeasurement.msg
  TrackingInfo.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS
  LIBRARIES msckf_vio image_processor
  CATKIN_DEPENDS
    roscpp std_msgs tf nav_msgs sensor_msgs geometry_msgs
    eigen_conversions tf_conversions random_numbers message_runtime
    image_transport cv_bridge message_filters pcl_conversions
    pcl_ros std_srvs
  DEPENDS Boost EIGEN3 OpenCV SUITESPARSE
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}  )
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories( ${SOPHUS_INCLUDE_DIR} )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories(${CHISEL_INCLUDE_DIR})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${REALSENSE2_INCLUDE_DIRS})

add_definitions(-DWITH_REALSENSE)


set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w")
set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3  -msse4.1 -Wall -std=c++14 -DSHADER_DIR=${GCFUSION_SHADER_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 -ftree-vectorize -lboost_system -lpthread -lboost_thread -lboost_filesystem -mavx2")

file(GLOB SOURCE_FILES
"./Flashfusion/*.cpp"
"./Flashfusion/*.h"
"./Flashfusion/Tools/*.h"
"./Flashfusion/Tools/*.cpp"
"./Flashfusion/GCSLAM/*.cpp"
"./Flashfusion/GCSLAM/*.h"
"./Flashfusion/GCSLAM/MILD/*.cpp"
"./Flashfusion/GCSLAM/MILD/*.h"
"./Flashfusion/GCSLAM/ORBSLAM/*.cpp"
"./Flashfusion/GCSLAM/ORBSLAM/*.h"
"./Flashfusion/GCFusion/*.cpp"
"./Flashfusion/GCFusion/*.h"
"./Flashfusion/Shaders/*.cpp"
"./Flashfusion/Shaders/*.h"
)

add_executable(FlashFusion ${SOURCE_FILES})
target_link_libraries(FlashFusion 
          ${catkin_LIBRARIES}
          ${OpenCV_LIBS} 
          ${Eigen_LIBRARIES}
          ${Pangolin_LIBRARIES}
          ${CHISEL_LIBRARY}
          ${OPENNI2_LIBRARY}
		      ${OPENGL_LIBRARIES} 
		      ${GLUT_LIBRARY}
          ${REALSENSE2_FOUND}
		      ${pthread} 
		      ${Boost_LIBRARIES}
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${OpenCV_INCLUDE_DIRS}
#   ${SUITESPARSE_INCLUDE_DIRS}
#   ${PCL_LIBRARY_DIRS}
#   include/lio
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lio.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# add_executable(vio_node vio/vio_node.cpp
#                             )

# target_link_libraries(vio_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lio.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
