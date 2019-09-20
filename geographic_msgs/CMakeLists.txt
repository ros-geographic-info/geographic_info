cmake_minimum_required(VERSION 3.5)
project(geographic_msgs)

find_package(ament_cmake REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
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
  "srv/GetGeographicMap.srv"
  "srv/GetGeoPath.srv"
  "srv/GetRoutePlan.srv"
  "srv/UpdateGeographicMap.srv"
  DEPENDENCIES unique_identifier_msgs geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
