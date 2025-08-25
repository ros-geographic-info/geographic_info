/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025 Roland Arsenault
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef GEODESY__ECEF_H_
#define GEODESY__ECEF_H_

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

    explicit ECEFPoint(const geometry_msgs::msg::Point & pt)
      : x(pt.x),
      y(pt.y),
      z(pt.z)
  {
    }

    explicit ECEFPoint(const geographic_msgs::msg::GeoPoint & pt);

    ECEFPoint(double _x, double _y, double _z)
      : x(_x),
      y(_y),
      z(_z)
  {
    }

    double x;  ///< X coordinate in meters.
    double y;  ///< Y coordinate in meters.
    double z;  ///< Z coordinate in meters.
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
  explicit ECEFPose(const geographic_msgs::msg::GeoPose & pose)
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

    ECEFPoint position;  ///< Position in ECEF coordinates.
    geometry_msgs::msg::Quaternion orientation;  ///< Orientation as a quaternion.
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

}  // namespace geodesy

#endif  // GEODESY__ECEF_H_
