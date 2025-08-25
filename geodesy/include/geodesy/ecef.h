#ifndef GEODESY_ECEF_H
#define GEODESY_ECEF_H

#include <limits>

#include "geodesy/wgs84.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace geodesy
{

/// Earth Centered Earth Fixed (ECEF) point.
  class ECEFPoint
  {
public:
  /** Null constructor. Makes an invalid point object. */
    ECEFPoint()
      : x(std::numeric_limits < double > ::quiet_NaN()),
      y(std::numeric_limits < double > ::quiet_NaN()),
      z(std::numeric_limits < double > ::quiet_NaN())
  {
    }

    ECEFPoint(const geometry_msgs::msg::Point & pt)
      : x(pt.x),
      y(pt.y),
      z(pt.z)
  {
    }

    ECEFPoint(const geographic_msgs::msg::GeoPoint & pt);

    ECEFPoint(double _x, double _y, double _z)
      : x(_x),
      y(_y),
      z(_z)
  {
    }

    double x; ///< X coordinate in meters.
    double y; ///< Y coordinate in meters.
    double z; ///< Z coordinate in meters.
  };


/// Earth Centered Earth Fixed (ECEF) pose.
  class ECEFPose
  {
public:
  /** Null constructor. Makes an invalid pose object. */
    ECEFPose()
      : position(),
      orientation()
  {
    }

  /** Create from a WGS 84 geodetic pose. */
    ECEFPose(const geographic_msgs::msg::GeoPose & pose)
      : position(pose.position),
      orientation(pose.orientation)
  {
    }


    ECEFPose(const ECEFPoint & pt, const geometry_msgs::msg::Quaternion & q)
      : position(pt),
      orientation(q)
  {
    }

  /** Create from a WGS 84 geodetic point and a quaternion. */
    ECEFPose(const geographic_msgs::msg::GeoPoint & pt,
           const geometry_msgs::msg::Quaternion & q)
      : position(pt),
      orientation(q)
  {
    }

    ECEFPoint position; ///< Position in ECEF coordinates.
    geometry_msgs::msg::Quaternion orientation; ///< Orientation as a quaternion.
  };


  void fromMsg(const geographic_msgs::msg::GeoPoint & from, ECEFPoint & to);
  void fromMsg(const geographic_msgs::msg::GeoPose & from, ECEFPose & to);
  geographic_msgs::msg::GeoPoint toMsg(const ECEFPoint & from);
  geographic_msgs::msg::GeoPose toMsg(const ECEFPose & from);


  bool isValid(const ECEFPoint & point);
  bool isValid(const ECEFPose & pose);


// Output stream operator for ECEF point.
  static inline std::ostream & operator << (std::ostream & out, const ECEFPoint & pt)
      {
      out << "(" << std::setprecision(10) << pt.x << ", "
      << pt.y << ", " << pt.z << ")";
      return out;
    }

    static inline std::ostream & operator << (std::ostream & out, const ECEFPose & pose)
      {
      out << pose.position << ", (["
      << std::setprecision(6)
      << pose.orientation.x << ", "
      << pose.orientation.y << ", "
      << pose.orientation.z << "], "
      << pose.orientation.w << ")";
      return out;
    }

/// @return a geometry Point corresponding to the ECEF Point.
    static inline geometry_msgs::msg::Point toGeometry(const ECEFPoint & from)
      {
      geometry_msgs::msg::Point to;
      to.x = from.x;
      to.y = from.y;
      to.z = from.z;
      return to;
    }

/// @return a geometry Pose corresponding to the ECEF Pose.
    static inline geometry_msgs::msg::Pose toGeometry(const ECEFPose & from)
      {
      geometry_msgs::msg::Pose to;
      to.position = toGeometry(from.position);
      to.orientation = from.orientation;
      return to;
    }

} // namespace geodesy

#endif // GEODESY_ECEF_H
