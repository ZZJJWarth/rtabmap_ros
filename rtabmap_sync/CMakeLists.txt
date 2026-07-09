cmake_minimum_required(VERSION 3.5)
project(rtabmap_sync)

set(ROBONIX_ENABLE_ZC OFF)
if(DEFINED ENV{ROBONIX_ENABLE_ZC} AND "$ENV{ROBONIX_ENABLE_ZC}" STREQUAL "1")
  set(ROBONIX_ENABLE_ZC ON)
endif()
message(STATUS "ROBONIX_ENABLE_ZC = ${ROBONIX_ENABLE_ZC}")

if(ROBONIX_ENABLE_ZC)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # issues #1285 #1288
  find_library(
    builtin_interfaces__rosidl_generator_c_LIB NAMES builtin_interfaces__rosidl_generator_c
    PATHS "/opt/ros/$ENV{ROS_DISTRO}/lib"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
  find_library(
    crypto_LIB NAMES crypto
    PATHS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED
  )
endif()

find_package(cv_bridge REQUIRED)
if(ROBONIX_ENABLE_ZC)
  find_package(cpp_pubsub REQUIRED)
endif()
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
if(ROBONIX_ENABLE_ZC)
  find_library(ROBONIX_ZC_LIBRARY NAMES robonix_zc REQUIRED)
endif()

option(RTABMAP_SYNC_MULTI_RGBD "Build with multi RGBD camera synchronization support"  OFF)
option(RTABMAP_SYNC_USER_DATA "Build with input user data support"  OFF)
MESSAGE(STATUS "RTABMAP_SYNC_MULTI_RGBD = ${RTABMAP_SYNC_MULTI_RGBD}")
MESSAGE(STATUS "RTABMAP_SYNC_USER_DATA  = ${RTABMAP_SYNC_USER_DATA}")
IF(RTABMAP_SYNC_MULTI_RGBD)
  add_definitions("-DRTABMAP_SYNC_MULTI_RGBD")
  SET(RTABMAP_SYNC_MULTI_RGBD_CONF 1)
ELSE()
  SET(RTABMAP_SYNC_MULTI_RGBD_CONF 0)
ENDIF()
IF(RTABMAP_SYNC_USER_DATA)
  add_definitions("-DRTABMAP_SYNC_USER_DATA")
  SET(RTABMAP_SYNC_USER_DATA_CONF 1)
ELSE()
  SET(RTABMAP_SYNC_USER_DATA_CONF 0)
ENDIF()

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

configure_file(cmake/extra_configs.cmake.in cmake/extra_configs.cmake IMMEDIATE @ONLY)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# libraries
SET(Libraries
   rclcpp_components::component
)
SET(PublicLibraries
    cv_bridge::cv_bridge
    rclcpp::rclcpp
    message_filters::message_filters
    image_transport::image_transport
    sensor_msgs::sensor_msgs
    nav_msgs::nav_msgs
    rtabmap_msgs::rtabmap_msgs
    rtabmap_conversions::rtabmap_conversions
    diagnostic_updater::diagnostic_updater
)

SET(AmentLibraries
   cv_bridge
   image_transport
   message_filters
   nav_msgs
   rclcpp
   rclcpp_components
   rtabmap_conversions
   rtabmap_msgs
   sensor_msgs
   diagnostic_updater
)
IF(ROBONIX_ENABLE_ZC)
  LIST(APPEND AmentLibraries cpp_pubsub)
ENDIF()

IF("$ENV{ROS_DISTRO}" STRLESS "lyrical")
  add_definitions(-DPRE_ROS_LYRICAL)
ENDIF()

###########
## Build ##
###########

SET(rtabmap_sync_lib_src
   src/CommonDataSubscriber.cpp
   src/impl/CommonDataSubscriberDepth.cpp
   src/impl/CommonDataSubscriberStereo.cpp
   src/impl/CommonDataSubscriberRGB.cpp
   src/impl/CommonDataSubscriberRGBD.cpp
   src/impl/CommonDataSubscriberRGBDX.cpp
   src/impl/CommonDataSubscriberScan.cpp
   src/impl/CommonDataSubscriberOdom.cpp
   src/impl/CommonDataSubscriberSensorData.cpp
)
IF(RTABMAP_SYNC_MULTI_RGBD)
  SET(rtabmap_sync_lib_src
    ${rtabmap_sync_lib_src}
    src/impl/CommonDataSubscriberRGBD2.cpp
    src/impl/CommonDataSubscriberRGBD3.cpp
    src/impl/CommonDataSubscriberRGBD4.cpp
    src/impl/CommonDataSubscriberRGBD5.cpp
    src/impl/CommonDataSubscriberRGBD6.cpp
  )
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
  
SET(rtabmap_sync_plugins_lib_src
   src/nodelets/rgbdx_sync.cpp
   src/nodelets/rgbd_sync.cpp
   src/nodelets/stereo_sync.cpp
   src/nodelets/rgb_sync.cpp
)


############################
## Declare a cpp library
############################
add_library(rtabmap_sync SHARED
   ${rtabmap_sync_lib_src}
)
if(ROBONIX_ENABLE_ZC)
  target_compile_features(rtabmap_sync PUBLIC cxx_std_17)
  target_compile_definitions(rtabmap_sync PRIVATE ROBONIX_ENABLE_ZC)
endif()
add_library(rtabmap_sync_plugins SHARED
   ${rtabmap_sync_plugins_lib_src}
)

target_include_directories(rtabmap_sync
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


if("$ENV{ROS_DISTRO}" STRLESS "lyrical")
  ament_target_dependencies(rtabmap_sync ${AmentLibraries})
  if(ROBONIX_ENABLE_ZC)
    target_link_libraries(rtabmap_sync ${ROBONIX_ZC_LIBRARY})
  endif()
ELSE()
  target_link_libraries(rtabmap_sync PRIVATE ${Libraries} PUBLIC ${PublicLibraries})
  if(ROBONIX_ENABLE_ZC)
    target_link_libraries(rtabmap_sync PRIVATE ${ROBONIX_ZC_LIBRARY})
    ament_target_dependencies(rtabmap_sync cpp_pubsub)
  endif()
  target_link_libraries(rtabmap_sync_plugins PUBLIC ${Libraries})
ENDIF()
target_link_libraries(rtabmap_sync_plugins PUBLIC rtabmap_sync)

rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBDSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::StereoSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBSync")
rclcpp_components_register_nodes(rtabmap_sync_plugins "rtabmap_sync::RGBDXSync")

add_executable(rtabmap_rgbd_sync src/RGBDSyncNode.cpp)
target_link_libraries(rtabmap_rgbd_sync PRIVATE rtabmap_sync_plugins)
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_rgbdx_sync src/RGBDXSyncNode.cpp)
target_link_libraries(rtabmap_rgbdx_sync PRIVATE rtabmap_sync_plugins)
set_target_properties(rtabmap_rgbdx_sync PROPERTIES OUTPUT_NAME "rgbdx_sync")

add_executable(rtabmap_stereo_sync src/StereoSyncNode.cpp)
target_link_libraries(rtabmap_stereo_sync PRIVATE rtabmap_sync_plugins)
set_target_properties(rtabmap_stereo_sync PROPERTIES OUTPUT_NAME "stereo_sync")

add_executable(rtabmap_rgb_sync src/RGBSyncNode.cpp)
target_link_libraries(rtabmap_rgb_sync PRIVATE rtabmap_sync_plugins)
set_target_properties(rtabmap_rgb_sync PROPERTIES OUTPUT_NAME "rgb_sync")

#############
## Install ##
#############
ament_export_dependencies(${AmentLibraries})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})          # To include downstream with targets
ament_export_libraries(rtabmap_sync rtabmap_sync_plugins) # To include downstream without targets

install(TARGETS 
   rtabmap_sync
   rtabmap_sync_plugins 
   EXPORT ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(TARGETS 
   rtabmap_rgbd_sync
   rtabmap_rgbdx_sync
   rtabmap_stereo_sync
   rtabmap_rgb_sync
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

ament_package(CONFIG_EXTRAS ${CMAKE_CURRENT_BINARY_DIR}/cmake/extra_configs.cmake)
