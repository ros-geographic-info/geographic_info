cmake_minimum_required(VERSION 3.5)
project(geographic_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uuid_msgs REQUIRED)

set(msg_files
  "msg/BoundingBox.msg"
  "msg/GeographicMapChanges.msg"
  "msg/GeographicMap.msg"
  "msg/GeoPath.msg"
  "msg/GeoPoint.msg"
  "msg/GeoPointStamped.msg"
  "msg/GeoPose.msg"
  "msg/GeoPoseStamped.msg"
  "msg/KeyValue.msg"
  "msg/MapFeature.msg"
  "msg/RouteNetwork.msg"
  "msg/RoutePath.msg"
  "msg/RouteSegment.msg"
  "msg/WayPoint.msg"
)

set(srv_files
  "srv/GetGeographicMap.srv"
  "srv/GetGeoPath.srv"
  "srv/GetRoutePlan.srv"
  "srv/UpdateGeographicMap.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  geometry_msgs
  std_msgs
  uuid_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
