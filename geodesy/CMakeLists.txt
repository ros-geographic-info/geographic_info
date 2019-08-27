cmake_minimum_required(VERSION 3.5)
project(geodesy)
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unique_id REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  angles
  geographic_msgs
  geometry_msgs
  sensor_msgs
  tf2
  unique_id
  unique_identifier_msgs
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# build C++ coordinate conversion library
add_library(geoconv STATIC src/conv/utm_conversions.cpp)
target_link_libraries(geoconv 
  ${dependencies}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}/)
install(TARGETS geoconv
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(geoconv)
ament_package()
