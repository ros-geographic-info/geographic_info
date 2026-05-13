^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geographic_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2026-05-13)
------------------

1.0.6 (2024-04-04)
------------------
* bump to 1.0.6 for release
* Add GeoPoint Validation (`#56 <https://github.com/ros-geographic-info/geographic_info/issues/56>`_)
  * Add GeoPoint Validation
  * Add ament_lint_common
  * Fix linter violations
  * Downgrade CMake to 3.13, remove GNUInstallDirs, Add licenses
  * Match upstream common_interfaces implementation
  * Change case to match ROS convention
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
  * Fix licensing problems
  * Only run the linters we need
  * Disable ament_copyright by not using ament_lint_common
  ---------
  Co-authored-by: Ryan Friedman <ryan.friedman@avinc.com>
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
* Contributors: Ryan, Steve Macenski

1.0.5 (2023-04-27)
------------------
* Bump to 1.0.5 (`#55 <https://github.com/ros-geographic-info/geographic_info/issues/55>`_)
  Co-authored-by: Ryan Friedman <ryan.friedman.con@avinc.com>
* Added GeoPoseWithCovariance and GeoPoseWithCovarianceStamped messages (`#51 <https://github.com/ros-geographic-info/geographic_info/issues/51>`_)
* Contributors: Ryan, chriswsuarez

1.0.4 (2020-06-01)
------------------
* bump to 1.0.4 (`#47 <https://github.com/ros-geographic-info/geographic_info/issues/47>`_)
* Contributors: Steve Macenski

1.0.3 (2020-05-29 14:18)
------------------------
* bump 1.0.3
* Contributors: stevemacenski

1.0.2 (2020-05-29 14:09)
------------------------
* bump to 1.0.2 for foxy release (`#44 <https://github.com/ros-geographic-info/geographic_info/issues/44>`_)
* adding ros1 bridge rules for message interfaces (`#32 <https://github.com/ros-geographic-info/geographic_info/issues/32>`_)
* Add compile flags from 23 (`#33 <https://github.com/ros-geographic-info/geographic_info/issues/33>`_)
  * add standard ROS 2 flags
  * remove extra semi-colon to suppress pedantic warning
* Contributors: Mikael Arguedas, Steve Macenski, Steven Macenski

1.0.1 (2019-09-20 16:36)
------------------------
* separately bumping 1.0.1 release (`#31 <https://github.com/ros-geographic-info/geographic_info/issues/31>`_)
* Contributors: Steven Macenski

1.0.0 (2019-09-20 16:13)
------------------------
* ROS2 port of geographic_msgs and geographic_info for robot_localization
* Contributors: Steven Macenski

0.5.3 (2018-03-27)
------------------

* Add additional information to ``GetGeoPath`` service response.

0.5.2 (2017-04-15)
------------------

* Fix merge error in GeoPath message.

0.5.1 (2017-04-15)
------------------

* Add GeoPath message with poses.
* Add GetGeoPath service (`#7`_).

0.5.0 (2017-04-07)
------------------

* Add new ``GeoPointStamped`` and ``GeoPoseStamped`` messages.

0.4.0 (2014-09-18)
------------------

0.3.2 (2014-08-30)
------------------

 * Add missing ``geometry_msgs`` dependency to ``catkin_package()`` 
   components in CMakeLists.txt (`#13`_), thanks to Timo Roehling.

0.3.1 (2013-10-03)
------------------

0.3.0 (2013-08-03)
------------------

 * Released to Hydro.
 * Convert to catkin build (`#3`_).

0.2.1 (2012-08-13)
------------------

 * Released to Fuerte.
 * Released to Groovy: 2013-03-27.

0.2.0 (2012-06-02)
------------------

 * Released to Electric.

0.1.0 (2012-04-10)
------------------

 * Initial release to Electric.

.. _`#3`: https://github.com/ros-geographic-info/geographic_info/issues/3
.. _`#6`: https://github.com/ros-geographic-info/geographic_info/issues/6
.. _`#7`: https://github.com/ros-geographic-info/geographic_info/issues/7
.. _`#13`: https://github.com/ros-geographic-info/geographic_info/pull/13
