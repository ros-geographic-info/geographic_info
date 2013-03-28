# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(geodesy)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED
  COMPONENTS
  angles
  geographic_msgs
  geometry_msgs
  sensor_msgs
  tf
  unique_id
  uuid_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()

# what other packages will need to use this one
catkin_package(
#    CATKIN-DEPENDS geographic_msgs geometry_msgs sensor_msgs tf unique_id uuid_msgs
    INCLUDE_DIRS include
    LIBRARIES geoconv
)

# build C++ coordinate conversion library
add_library(geoconv src/conv/utm_conversions.cpp)
target_link_libraries(geoconv ${catkin_LIBRARIES})

# unit tests
catkin_add_gtest(test_wgs84 tests/test_wgs84.cpp)
target_link_libraries(test_wgs84 ${catkin_LIBRARIES})
catkin_add_gtest(test_utm tests/test_utm.cpp)
target_link_libraries(test_utm geoconv ${catkin_LIBRARIES})
catkin_add_nosetests(tests/test_bounding_box.py)
catkin_add_nosetests(tests/test_props.py)
catkin_add_nosetests(tests/test_utm.py)
catkin_add_nosetests(tests/test_uuid.py)
catkin_add_nosetests(tests/test_wu_point.py)

install(
  DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  TARGETS geoconv
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
