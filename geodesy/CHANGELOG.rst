^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geodesy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2026-05-13)
------------------
* Add ECEF support as well as WGS84 geodesics calculations. (`#60 <https://github.com/ros-geographic-info/geographic_info/issues/60>`_)
  * Add ECEF support as well as WGS84 geodesics calculations.
  * Add documentation, tests and uncrustify
  * Fix ament_cpplint issues
* Fix targets (`#59 <https://github.com/ros-geographic-info/geographic_info/issues/59>`_)
* [kilted] Update deprecated call to ament_target_dependencies (`#58 <https://github.com/ros-geographic-info/geographic_info/issues/58>`_)
* Contributors: David V. Lu!!, Roland Arsenault

1.0.6 (2024-04-04)
------------------
* bump to 1.0.6 for release
* Contributors: Steve Macenski

1.0.5 (2023-04-27)
------------------
* Bump to 1.0.5 (`#55 <https://github.com/ros-geographic-info/geographic_info/issues/55>`_)
  Co-authored-by: Ryan Friedman <ryan.friedman.con@avinc.com>
* Contributors: Ryan

1.0.4 (2020-06-01)
------------------
* bump to 1.0.4 (`#47 <https://github.com/ros-geographic-info/geographic_info/issues/47>`_)
* Update python3 pyproj (`#46 <https://github.com/ros-geographic-info/geographic_info/issues/46>`_)
* Contributors: Steve Macenski

1.0.3 (2020-05-29 14:18)
------------------------
* bump 1.0.3
* Update python-catkin-pkg to python3-catkin-pkg (`#45 <https://github.com/ros-geographic-info/geographic_info/issues/45>`_)
* Contributors: Steve Macenski, stevemacenski

1.0.2 (2020-05-29 14:09)
------------------------
* bump to 1.0.2 for foxy release (`#44 <https://github.com/ros-geographic-info/geographic_info/issues/44>`_)
* Added an option to force zones on conversionfrom WGS84 to utm. This will allow for smoother transitions across zonal boundaries (`#43 <https://github.com/ros-geographic-info/geographic_info/issues/43>`_)
  Co-authored-by: Peter Milani <peter@emesent.io>
* Python3 fixes (`#35 <https://github.com/ros-geographic-info/geographic_info/issues/35>`_)
  * Python3 fixes
  * fixed tests
* Add compile flags from 23 (`#33 <https://github.com/ros-geographic-info/geographic_info/issues/33>`_)
  * add standard ROS 2 flags
  * remove extra semi-colon to suppress pedantic warning
* Contributors: Andreas Klintberg, Mikael Arguedas, Peter, Steve Macenski

1.0.1 (2019-09-20 16:36)
------------------------
* separately bumping 1.0.1 release (`#31 <https://github.com/ros-geographic-info/geographic_info/issues/31>`_)
* Remove rosunit from testing in geodesy & remove experimental API claim on ROS2 branch (`#30 <https://github.com/ros-geographic-info/geographic_info/issues/30>`_)
  * remove experimental warning, its been pretty stable for a long time
  * removing rosunit from package.xml
* Contributors: Steven Macenski

1.0.0 (2019-09-20 16:13)
------------------------
* ROS2 port of geographic_msgs and geographic_info for robot_localization
* Contributors: Steven Macenski

0.5.3 (2018-03-27)
------------------

0.5.2 (2017-04-15)
------------------

0.5.1 (2017-04-15)
------------------

0.5.0 (2017-04-07)
------------------

 * Removed Python setup install_requires option.

0.4.0 (2014-09-18)
------------------

 * Released to Indigo.
 * Remove deprecated geodesy.gen_uuid module (`#4`_).

0.3.2 (2014-08-30)
------------------

 * Released to Indigo.
 * Make C++ template declaration for WGS-84 header compatible with GCC
   4.7 compiler (`#12`_), thanks to Mike Purvis and Dirk Thomas.

0.3.1 (2013-10-03)
------------------

 * Fix ROS Hydro header install problem (`#9`_).

0.3.0 (2013-08-03)
------------------

 * Released to Hydro.
 * Convert to catkin build (`#3`_).
 * Remove unnecessary roscpp and rospy dependencies (`#6`_).

0.2.1 (2012-08-13)
------------------

 * Released to Fuerte.
 * Released to Groovy: 2013-03-27.
 * Use unique_identifier for UUID interfaces.

0.2.0 (2012-06-02)
------------------

 * Many new mapping messages and services added.
 * Use PROJ.4 library for Python geodesy API (C++ API not ported to
   PROJ.4 yet).
 * Released to Electric.

0.1.0 (2012-04-10)
------------------

 * Initial release to Electric.

.. _`#3`: https://github.com/ros-geographic-info/geographic_info/issues/3
.. _`#4`: https://github.com/ros-geographic-info/geographic_info/issues/4
.. _`#6`: https://github.com/ros-geographic-info/geographic_info/issues/6
.. _`#9`: https://github.com/ros-geographic-info/geographic_info/issues/9
.. _`#12`: https://github.com/ros-geographic-info/geographic_info/issues/12
