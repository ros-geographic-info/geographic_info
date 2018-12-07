# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(geodesy)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unique_id REQUIRED)
find_package(uuid_msgs REQUIRED)
  
include_directories(
        include 
        ${angles_INCLUDE_DIRS} 
        ${tf2_INCLUDE_DIRS} 
        ${geometry_msgs_INCLUDE_DIRS} 
        ${geographic_msgs_INCLUDE_DIRS}
)

ament_python_install_package(${PROJECT_NAME} 
        PACKAGE_DIR src/${PROJECT_NAME}
)

# build C++ coordinate conversion library
add_library(geoconv STATIC src/conv/utm_conversions.cpp)
target_link_libraries(geoconv 
  ${std_msgs_LIBRARIES}
  ${angles_LIBRARIES}
  ${geographic_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${unique_id_LIBRARIES}
  ${uuid_msgs_LIBRARIES}
)

ament_export_dependencies(
        geographic_msgs
        geometry_msgs
        sensor_msgs
        tf2
        unique_id
        uuid_msgs
)

ament_export_include_directories(include)
ament_export_libraries(geoconv)

ament_package()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}/)
install(TARGETS geoconv
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)