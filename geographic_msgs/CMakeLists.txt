cmake_minimum_required(VERSION 2.8.3)
project(geographic_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  uuid_msgs
)

# is this required?
## We want boost/format.hpp, which isn't in its own component.
#find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  BoundingBox.msg
  GeographicMapChanges.msg
  GeographicMap.msg
  GeoPoint.msg
  GeoPose.msg
  KeyValue.msg
  MapFeature.msg
  RouteNetwork.msg
  RoutePath.msg
  RouteSegment.msg
  WayPoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetGeographicMap.srv
  GetRoutePlan.srv
  UpdateGeographicMap.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  uuid_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime uuid_msgs std_msgs
  DEPENDS Boost)
