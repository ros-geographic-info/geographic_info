cmake_minimum_required(VERSION 3.5)
project(geodesy)
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  angles
  geographic_msgs
  geometry_msgs
  sensor_msgs
  unique_identifier_msgs
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

# build C++ coordinate conversion library
add_library(geoconv STATIC src/conv/utm_conversions.cpp)
ament_target_dependencies(geoconv 
  ${dependencies}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}/)
install(TARGETS geoconv
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(geoconv)
ament_package()
