/* -*- mode: C++ -*- */
/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2011 Jack O'Quin
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

#ifndef _GEODETIC_SYSTEM_H_
#define _GEODETIC_SYSTEM_H_

#include <limits>
#include <ctype.h>
#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>

/** @file

    @brief Geodetic system for ROS latitude and longitude messages

    Standard ROS lat/long coordinates are defined in terms of the
    World Geodetic System (WGS 84) ellipsoid used by most navigation
    satellite receivers.

    Many other geodetic coordinate systems can be defined.  They
    should always be converted to WGS 84 when publishing ROS messages
    to avoid confusion among subscribers.
    
    @author Jack O'Quin
 */

namespace geodesy
{

  /** @return true if no altitude specified. */
  static inline bool is2D(const geographic_msgs::GeoPoint &pt)
  {
    return (pt.altitude != pt.altitude);
  }

  /** @return true if pose has no altitude. */
  static inline bool is2D(const geographic_msgs::GeoPose &pose)
  {
    return is2D(pose.position);
  }

  /** @return true if point is valid. */
  static inline bool isValid(const geographic_msgs::GeoPoint &pt)
  {
    if (pt.latitude < -90.0 || pt.latitude > 90.0)
      return false;

    if (pt.longitude < -180.0 || pt.longitude >= 180.0)
      return false;

    return true;
  }

  /** @return true if point is valid. */
  static inline bool isValid(const geographic_msgs::GeoPose &pose)
  {
    /// @todo validate orientation quaternion (should be normalized)
    return isValid(pose.position);
  }

  /** normalize
   *
   *  @param pt point to be normalized
   *
   *  Normalizes the longitude to [-180, 180).
   *  Clamps latitude to [-90, 90].
   */
  static inline void normalize(geographic_msgs::GeoPoint &pt)
  {
    pt.longitude = (fmod(fmod((pt.longitude + 180.0), 360.0) + 360.0,
                         360.0)
                    - 180.0);
    pt.latitude = std::min(std::max(pt.latitude, -90.0), 90.0);
  }

}; // namespace geodesy

#endif // _GEODETIC_SYSTEM_H_
