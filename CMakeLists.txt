cmake_minimum_required(VERSION 2.8.3)
project(r2dio)
#add_definitions(-w)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
  cv_bridge
  image_transport
)

find_package(Eigen3)
find_package(OpenCV 4.2 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  include
  include/peac
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)

add_executable(r2dio_laser_processing_node  src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/param.cpp src/utils.cpp )
target_link_libraries(r2dio_laser_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(r2dio_odom_estimation_node src/utils.cpp src/odomEstimationNode.cpp src/param.cpp src/odomEstimationClass.cpp src/imuOptimizationFactor.cpp src/lidarOptimizationFactor.cpp src/imuPreintegrationClass.cpp src/param.cpp src/poseOptimizationFactor.cpp)
target_link_libraries(r2dio_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(r2dio_laser_mapping_node src/laserMappingNode.cpp src/utils.cpp src/laserMappingClass.cpp src/param.cpp)
target_link_libraries(r2dio_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(trajectory_listener src/tools/trajectory_listener.cpp)
target_link_libraries(trajectory_listener ${catkin_LIBRARIES})
