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

#ifndef _UTMPOINT_H_
#define _UTMPOINT_H_

#include <limits>
#include <ctype.h>
#include <ros/ros.h>
#include <geodesy/geodetic_system.h>

/** @file

    @brief Universal Transverse Mercator coordinates

    For outdoor robotics applications, Euclidean projections like UTM
    are easier to work with than latitude and longitude.  This system
    is slightly more general than strict UTM.  It is based on the
    Military Grid Reference System (MGRS), which can be extended to
    cover the poles, allowing well-defined transformations for every
    latitude and longitude.

    @todo add Universal Polar Stereographic support

    @author Jack O'Quin
 */

namespace geodesy
{

/** Universal Transverse Mercator (UTM) point
 *
 *  This representation is more general than pure UTM.  By including
 *  the top-level grid zone designator (GZD) from the Military Grid
 *  Reference System (MGRS), it allows unambiguous use of Universal
 *  Polar Stereographic (UPS) coordinates for the polar regions not
 *  covered by UTM.
 */
class UTMPoint
{
 public:

  /** null constructor */
  UTMPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(0),
    band(' ')
  {}

  /** copy constructor */
  UTMPoint(const UTMPoint &that):
    easting(that.easting),
    northing(that.northing),
    altitude(that.altitude),
    zone(that.zone),
    band(that.band)
  {}
  
  /** create from GeoPoint message */
  UTMPoint(geographic_msgs::GeoPoint latlong);

  /** create 2-D grid point */
  UTMPoint(double _easting, double _northing, uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(_zone),
    band(_band)
  {}

  /** create 3-D grid point */
  UTMPoint(double _easting, double _northing, double _altitude,
            uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(_altitude),
    zone(_zone),
    band(_band)
  {}

  // data members
  double easting;           ///< easting within zone [meters]
  double northing;          ///< northing within zone [meters] 
  double altitude;          ///< altitude [meters], NaN if unspecified
  uint8_t zone;             ///< UTM longitude zone number
  char   band;              ///< MGRS latitude band letter

}; // class UTMPoint

/** Universal Transverse Mercator (UTM) pose
 *
 *  This representation is more general than pure UTM.  By including
 *  the top-level grid zone designator (GZD) from the Military Grid
 *  Reference System (MGRS), it allows unambiguous use of Universal
 *  Polar Stereographic (UPS) coordinates for the polar regions not
 *  covered by UTM.
 *
 *  @todo add Universal Polar Stereographic support
 */
class UTMPose
{
 public:

  /** null constructor */
  UTMPose():
    position(),
    orientation()
  {}

  /** copy constructor */
  UTMPose(const UTMPose &that):
    position(that.position),
    orientation(that.orientation)
  {}

  /** create from a UTMPoint message and a quaternion */
  UTMPose(UTMPoint pt,
          geometry_msgs::Quaternion q):
    position(pt),
    orientation(q)
  {}
  
  /** create from a GeoPose message */
  UTMPose(geographic_msgs::GeoPose pose);

  /** create from a GeoPoint message and a quaternion */
  UTMPose(geographic_msgs::GeoPoint pt,
          geometry_msgs::Quaternion q):
    position(pt),
    orientation(q)
  {}

  // data members
  UTMPoint position;
  geometry_msgs::Quaternion orientation;

}; // class UTMPose

/** transform UTM point to geographic point */
geographic_msgs::GeoPoint fromUTMPoint(const UTMPoint &pt);

/** transform UTM pose to geographic point */
geographic_msgs::GeoPose fromUTMPose(const UTMPose &pt);

/** return true if no altitude specified */
bool isFlat(const UTMPoint &pt)
{
  // true if altitude is a NaN
  return (pt.altitude != pt.altitude);
}

/** return true if no altitude specified */
bool isFlat(const UTMPose &pose)
{
  // true if position has no altitude
  return isFlat(pose.position);
}

/** return true if point is valid */
bool isValid(const UTMPoint &pt)
{
  if (pt.zone < 1 || pt.zone > 60)
    return false;

  if (!isupper(pt.band) || pt.band == 'I' || pt.band == 'O')
    return false;

  // The Universal Polar Stereographic bands are not currently
  // supported.  When they are: A, B, Y and Z will be valid (and the
  // zone number will be ignored).
  if (pt.band < 'C' || pt.band > 'X')
    return false;

  return true;
}

/** return true if pose is valid */
bool isValid(const UTMPose &pose)
{
  if (!isValid(pose.position))
    return false;

  /// @todo validate orientation quaternion (should be normalized)

  return true;
}

/** transform geographic point to UTM point */
UTMPoint toUTMPoint(const geographic_msgs::GeoPoint &pt);

/** transform geographic pose to UTM pose */
UTMPose toUTMPose(const geographic_msgs::GeoPose &pt);

}; // namespace geodesy

#endif // _UTMPOINT_H_
