
cmake_minimum_required (VERSION 3.8)

project (my_topic)
 
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(ROS_INC_DIR	   "/opt/ros/galactic/include")
set(ROS_LIB_DIR	   "/opt/ros/galactic/lib")

add_executable(publisher src/publisher.cpp)
add_executable(subscriber src/subscriber.cpp)

target_include_directories(publisher PRIVATE ${ROS_INC_DIR})
target_include_directories(subscriber PRIVATE ${ROS_INC_DIR})

# my lib
add_library(BASE64 STATIC src/base64.cpp src/base64.hpp)
add_library(TIMER STATIC src/timer.cpp src/timer.hpp)
add_library(PICTURE STATIC src/picture.cpp src/picture.hpp)


find_package( OpenCV 4.5 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(publisher PRIVATE   ${ROS_LIB_DIR}/librclcpp.so
                                          #${ROS_LIB_DIR}/librclcpp_lifecycle.so
                                          ${ROS_LIB_DIR}/librcl.so
                                          ${ROS_LIB_DIR}/librcutils.so
                                          #${ROS_LIB_DIR}/libtf2.so
                                          #${ROS_LIB_DIR}/libtf2_ros.so
                                          #${ROS_LIB_DIR}/libtf2_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libbuiltin_interfaces__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libsensor_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libgeometry_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libstd_srvs__rosidl_typesupport_cpp.so
                                          ${ROS_LIB_DIR}/libstd_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/x86_64-linux-gnu/libconsole_bridge.so
                                          #${ROS_LIB_DIR}/libclass_loader.so
                                          ${ROS_LIB_DIR}/libtracetools.so
                                          ${ROS_LIB_DIR}/librmw.so
                                          ${ROS_LIB_DIR}/liblibstatistics_collector.so
                                          ${ROS_LIB_DIR}/libstatistics_msgs__rosidl_typesupport_cpp.so
                                          pthread
                                          BASE64
                                          ${OpenCV_LIBS}
                                          TIMER
                                          PICTURE
                                          )

target_link_libraries(subscriber PRIVATE  ${ROS_LIB_DIR}/librclcpp.so
                                          #${ROS_LIB_DIR}/librclcpp_lifecycle.so
                                          ${ROS_LIB_DIR}/librcl.so
                                          ${ROS_LIB_DIR}/librcutils.so
                                          #${ROS_LIB_DIR}/libtf2.so
                                          #${ROS_LIB_DIR}/libtf2_ros.so
                                          #${ROS_LIB_DIR}/libtf2_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libbuiltin_interfaces__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libsensor_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libgeometry_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/libstd_srvs__rosidl_typesupport_cpp.so
                                          ${ROS_LIB_DIR}/libstd_msgs__rosidl_typesupport_cpp.so
                                          #${ROS_LIB_DIR}/x86_64-linux-gnu/libconsole_bridge.so
                                          #${ROS_LIB_DIR}/libclass_loader.so
                                          ${ROS_LIB_DIR}/libtracetools.so
                                          ${ROS_LIB_DIR}/librmw.so
                                          ${ROS_LIB_DIR}/liblibstatistics_collector.so
                                          ${ROS_LIB_DIR}/libstatistics_msgs__rosidl_typesupport_cpp.so
                                          pthread 
                                          BASE64
                                          ${OpenCV_LIBS}
                                          TIMER
                                          PICTURE
                                          )

