cmake_minimum_required(VERSION 3.0.0)
project(agv_controller VERSION 0.1.0)

# set(CMAKE_BUILD_TYPE "Debug")
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


#Cartographer depended
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
set(CARTOGRAPHER_GMOCK_LIBRARIES ${GMOCK_LIBRARIES})

add_definitions(
-DAGV_ROS_DEBUG
)
#Base include 
include_directories(
  /usr/include/eigen3
  /usr/include/
  /usr/local/include/
  /usr/local/include/canfestival
  /usr/local/include/apriltag
  #Camera
  /opt/MVS/include

  include
  include/agv_common
  include/agv_map_ctrl
  include/agv_net_ctrl
  include/agv_ros_ctrl
  
  include/agv_slammer
  include/agv_slammer/amcl
  include/agv_slammer/amcl/map
  include/agv_slammer/amcl/pf
  include/agv_slammer/amcl/sensors
  include/agv_slammer/safety

  include/agv_chassis_ctrl
  include/agv_sensor
  include/agv_sensor/sick_scan
  include/agv_tf
  include/agv_ctrl
  include/agv_rvd_ctrl
)

LINK_DIRECTORIES(
  "/opt/MVS/lib/64"
)

#Main Program Generate!!
aux_source_directory(./src DIR_SRCS) #aux_source_directory的缺点在于，对新加入的文件，因为不修改CMakeLists.txt，导致检测不到更改，此时需重新CMake
aux_source_directory(./src/agv_common DIR_AGV_COMMON)
aux_source_directory(./src/agv_map_ctrl DIR_AGV_MAP_CTRL)
aux_source_directory(./src/agv_net_ctrl DIR_AGV_NET_CTRL)
aux_source_directory(./src/agv_ros_ctrl DIR_AGV_ROS_CTRL)

aux_source_directory(./src/agv_slammer DIR_AGV_SLAMMER)
aux_source_directory(./src/agv_slammer/amcl DIR_AGV_SLAMMER_AMCL)
aux_source_directory(./src/agv_slammer/amcl/map DIR_AGV_SLAMMER_AMCL_MAP)
aux_source_directory(./src/agv_slammer/amcl/pf DIR_AGV_SLAMMER_AMCL_PF)
aux_source_directory(./src/agv_slammer/amcl/sensors DIR_AGV_SLAMMER_AMCL_SENSORS)
aux_source_directory(./src/agv_slammer/safety DIR_AGV_SLAMMER_SAFETY)

aux_source_directory(./src/agv_chassis_ctrl DIR_AGV_CHASSIS_CTRL)
aux_source_directory(./src/agv_sensor/sick_scan DIR_AGV_SENSOR_SICK_SCAN)
aux_source_directory(./src/agv_sensor/sick_scan/cola2 DIR_AGV_SENSOR_SICK_SCAN_COLA2)
aux_source_directory(./src/agv_sensor/sick_scan/communication DIR_AGV_SENSOR_SICK_SCAN_COMM)
aux_source_directory(./src/agv_sensor/sick_scan/data_processing DIR_AGV_SENSOR_SICK_SCAN_DATA_PROCESSING)
aux_source_directory(./src/agv_sensor/sick_scan/datastructure DIR_AGV_SENSOR_SICK_SCAN_DATASTRUCTURE)
aux_source_directory(./src/agv_tf DIR_AGV_TF)
aux_source_directory(./src/agv_ctrl DIR_AGV_CTRL)
aux_source_directory(./src/agv_rvd_ctrl DIR_AGV_RVD)

add_executable(${PROJECT_NAME}
  ${DIR_SRCS}
  ${DIR_AGV_COMMON}
  ${DIR_AGV_ROS_CTRL}
  ${DIR_AGV_MAP_CTRL}
  ${DIR_AGV_NET_CTRL}

  ${DIR_AGV_SLAMMER}
  ${DIR_AGV_SLAMMER_AMCL}
  ${DIR_AGV_SLAMMER_AMCL_MAP}
  ${DIR_AGV_SLAMMER_AMCL_PF}
  ${DIR_AGV_SLAMMER_AMCL_SENSORS}
  ${DIR_AGV_SLAMMER_SAFETY}

  ${DIR_AGV_CHASSIS_CTRL}
  ${DIR_AGV_RVD}

  ${DIR_AGV_SENSOR_SICK_SCAN}
  ${DIR_AGV_SENSOR_SICK_SCAN_COLA2}
  ${DIR_AGV_SENSOR_SICK_SCAN_COMM}
  ${DIR_AGV_SENSOR_SICK_SCAN_DATA_PROCESSING}
  ${DIR_AGV_SENSOR_SICK_SCAN_DATASTRUCTURE}

  ${DIR_AGV_TF}
  ${DIR_AGV_CTRL}
)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

#Cartographer
#target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)
#target_link_libraries(${PROJECT_NAME} PUBLIC pthread)
#target_link_libraries(${PROJECT_NAME} PUBLIC libgflags.a)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  rt
  glog
  protobuf
  ceres
  libgflags.a
  cartographer
  canfestival
  canfestival_unix
  canfestival_can_socket
  dl
  pthread
  boost_thread

  MvCameraControl
  apriltag
  small_scara_interface
)