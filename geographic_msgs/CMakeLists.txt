cmake_minimum_required(VERSION 3.5)
project(geographic_msgs)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuid_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

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
  DEPENDENCIES 
  builtin_interfaces
  std_msgs
  geometry_msgs 
  uuid_msgs
)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} ${uuid_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${uuid_msgs_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${uuid_msgs_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(uuid_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
