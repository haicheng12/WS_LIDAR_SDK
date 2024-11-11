cmake_minimum_required(VERSION 2.8.3)
project(ws_30pcd_et3)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8 )  
set(CMAKE_CXX_STANDARD 11)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  pcl_ros
  message_generation
  visualization_msgs
)

find_package(PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)  #指定pcl1.8路径

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/pcl-1.8
)

link_directories(${PCL_LIBRARY_DIRS})

add_service_files(
  FILES
  state.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES service_client_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

add_executable(scan_frame 
  src/mission/mission.cpp
  src/task/task.cpp
  src/task/points_task.cpp
  src/task/imu_task.cpp
  src/task/scan_task.cpp
  src/udp/udp.cpp
  src/main.cpp)
target_link_libraries(scan_frame ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(state_client src/state_client.cpp)
target_link_libraries(state_client ${catkin_LIBRARIES})

add_executable(calculate_imu src/calculate_imu.cpp)
target_link_libraries(calculate_imu ${catkin_LIBRARIES})

install(TARGETS scan_frame
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS state_client
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS calculate_imu
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
		 
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

install(DIRECTORY srv/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv)
