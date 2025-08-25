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

#include <cmath>
#include "geodesy/ecef.h"
#include "geodesy/wgs84_ellipsoid.h"

namespace geodesy
{

void fromMsg(const geographic_msgs::msg::GeoPoint & from, ECEFPoint & to)
{
  // Convert WGS 84 geodetic point to ECEF coordinates
  // Calculation details can be found in the following sources:
  // - https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
  // - B. Hofmann-Wellenhof; H. Lichtenegger; J. Collins (1997). GPS - theory and practice.
  //   Section 10.2.1. p. 282. ISBN 3-211-82839-7

  double lat_rad = from.latitude * M_PI / 180.0;
  double lon_rad = from.longitude * M_PI / 180.0;
  double alt = from.altitude;
  if (alt != alt) {  // Check if altitude is NaN
    alt = 0.0;  // Default to zero if altitude is not specified
  }
  auto n = wgs84::Ellipsoid::N(lat_rad);

  to.x = (n + alt) * cos(lat_rad) * cos(lon_rad);
  to.y = (n + alt) * cos(lat_rad) * sin(lon_rad);
  to.z = (n * (1.0 - wgs84::Ellipsoid::Parameters::e2) + alt) * sin(lat_rad);
}

void fromMsg(const geographic_msgs::msg::GeoPose & from, ECEFPose & to)
{
  // Convert WGS 84 geodetic pose to ECEF pose
  fromMsg(from.position, to.position);
  to.orientation = from.orientation;
}

geographic_msgs::msg::GeoPoint toMsg(const ECEFPoint & from)
{
  // Convert ECEF coordinates to WGS 84 geodetic point
  // Calculation details can be found in the following sources:
  // - https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
  // - B. Hofmann-Wellenhof; H. Lichtenegger; J. Collins (1997). GPS - theory and practice.
  //   Section 10.2.1. p. 282. ISBN 3-211-82839-7

  geographic_msgs::msg::GeoPoint pt;

  double ep2 = (wgs84::Ellipsoid::Parameters::a * wgs84::Ellipsoid::Parameters::a) /
    (wgs84::Ellipsoid::Parameters::b * wgs84::Ellipsoid::Parameters::b) -1.0;
  double r = sqrt(from.x * from.x + from.y * from.y);
  double E2 = wgs84::Ellipsoid::Parameters::a * wgs84::Ellipsoid::Parameters::a -
    wgs84::Ellipsoid::Parameters::b * wgs84::Ellipsoid::Parameters::b;
  double F = 54 * wgs84::Ellipsoid::Parameters::b * wgs84::Ellipsoid::Parameters::b * from.z *
    from.z;
  double G = r * r + (1.0 - wgs84::Ellipsoid::Parameters::e2) * from.z * from.z -
    wgs84::Ellipsoid::Parameters::e2 * E2;
  double C = (wgs84::Ellipsoid::Parameters::e2 * wgs84::Ellipsoid::Parameters::e2 * F * r * r) /
    (G * G * G);
  double s = pow(1.0 + C + sqrt(C * C + 2 * C), 1.0 / 3.0);
  double P = F / (3.0 * pow((s + (1.0 / s) + 1.0), 2.0) * G * G);
  double Q = sqrt(1.0 + 2.0 * wgs84::Ellipsoid::Parameters::e2 * wgs84::Ellipsoid::Parameters::e2 *
      P);
  double r0 = (-(P * wgs84::Ellipsoid::Parameters::e2 * r) / (1.0 + Q)) +
    sqrt((1.0 / 2.0) * wgs84::Ellipsoid::Parameters::a * wgs84::Ellipsoid::Parameters::a *
      (1.0 + 1.0 / Q) -
      ((P * (1 - wgs84::Ellipsoid::Parameters::e2) * from.z * from.z) / (Q * (1.0 + Q))) -
      (1.0 / 2.0) * P * r * r);
  double U = sqrt(pow(r - wgs84::Ellipsoid::Parameters::e2 * r0, 2.0) + from.z * from.z);
  double V = sqrt(pow(r - wgs84::Ellipsoid::Parameters::e2 * r0,
      2.0) + (1.0 - wgs84::Ellipsoid::Parameters::e2) * from.z * from.z);
  double Z0 = wgs84::Ellipsoid::Parameters::b * wgs84::Ellipsoid::Parameters::b * from.z /
    (wgs84::Ellipsoid::Parameters::a * V);
  pt.altitude = U *
    (1.0 - (wgs84::Ellipsoid::Parameters::b * wgs84::Ellipsoid::Parameters::b) /
    (wgs84::Ellipsoid::Parameters::a * V));
  pt.latitude = atan((from.z + ep2 * Z0) / r);
  pt.longitude = atan2(from.y, from.x);
  pt.latitude = pt.latitude * 180.0 / M_PI;  // Convert to degrees
  pt.longitude = pt.longitude * 180.0 / M_PI;  // Convert to degrees
  // Normalize longitude
  pt.longitude = fmod(fmod(pt.longitude + 180.0, 360.0) + 360.0, 360.0) - 180.0;
  pt.latitude = std::min(std::max(pt.latitude, -90.0), 90.0);  // Clamp latitude

  return pt;
}

geographic_msgs::msg::GeoPose toMsg(const ECEFPose & from)
{
  // Convert ECEF pose to WGS 84 geodetic pose
  geographic_msgs::msg::GeoPose pose;
  pose.position = toMsg(from.position);
  pose.orientation = from.orientation;
  return pose;
}

ECEFPoint::ECEFPoint(const geographic_msgs::msg::GeoPoint & pt)
{
  fromMsg(pt, *this);
}

bool isValid(const ECEFPoint & pt)
{
  // Check if any coordinate is NaN
  return !(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z));
}

bool isValid(const ECEFPose & pose)
{
  // Check if position is valid and orientation quaternion is normalized
  double len2 = (pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z +
    pose.orientation.w * pose.orientation.w);
  return isValid(pose.position) && fabs(len2 - 1.0) <= std::numeric_limits<double>::epsilon();
}


}  // namespace geodesy
