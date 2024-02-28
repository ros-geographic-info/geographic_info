cmake_minimum_required(VERSION 3.13)
project(geographic_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  "msg/GeoPoseWithCovariance.msg"
  "msg/GeoPoseWithCovarianceStamped.msg"
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

add_library(validation INTERFACE)
add_library(${PROJECT_NAME}::validation ALIAS validation)
add_subdirectory(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

install(
  FILES geographic_msgs_mapping_rule.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS validation
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
