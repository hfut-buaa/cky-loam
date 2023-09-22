cmake_minimum_required(VERSION 2.8.3)
project(ckyloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(ALL_TARGET_LIBRARIES "")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)
find_package(G2O REQUIRED)


list(APPEND ALL_TARGET_LIBRARIES 
      ${G2O_TYPES_DATA}
      ${G2O_CORE_LIBRARY}
      ${G2O_STUFF_LIBRARY}
      ${G2O_SOLVER_PCG}
      ${G2O_SOLVER_CSPARSE}
      ${G2O_SOLVER_CHOLMOD}
      ${G2O_TYPES_SLAM3D}
      ${G2O_TYPES_SLAM3D_ADDONS}
      ${CHOLMOD_LIBRARIES}
      #SOPHUS
      #${Sophus_LIBRARIES}
      )
INCLUDE_DIRECTORIES("/usr/include/eigen3")
#include_directories(${G2O_INCLUDE_DIRS})
include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(frameFeature src/frameFeature.cpp)
target_link_libraries(frameFeature ${catkin_LIBRARIES} ${PCL_LIBRARIES}  -lgtsam -ltbb)

add_executable(lidarOdometry src/lidarOdometry.cpp)
target_link_libraries(lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}  -lgtsam -ltbb)

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${ALL_TARGET_LIBRARIES}  -lgtsam -ltbb)

