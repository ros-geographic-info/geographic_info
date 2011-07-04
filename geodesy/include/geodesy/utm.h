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

/** Universal Transverse Mercator (UTM) point.
 *
 *  The @c altitude may or may not be specified, making the point
 *  either 3D or 2D (flattened). The @c altitude of a 2D point is not
 *  a number (NaN).
 *
 *  Including the top-level grid zone designator (GZD) from the
 *  Military Grid Reference System (MGRS) permits unambiguous use of
 *  Universal Polar Stereographic (UPS) coordinates for the polar
 *  regions not covered by UTM, making this representation more
 *  general than pure UTM.
 */
class UTMPoint
{
 public:

  /** Null constructor. Makes a 2D, invalid point object. */
  UTMPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(0),
    band(' ')
  {}

  /** Copy constructor. */
  UTMPoint(const UTMPoint &that):
    easting(that.easting),
    northing(that.northing),
    altitude(that.altitude),
    zone(that.zone),
    band(that.band)
  {}
  
  UTMPoint(const geographic_msgs::GeoPoint &pt);

  /** Create a flattened 2-D grid point. */
  UTMPoint(double _easting, double _northing, uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(_zone),
    band(_band)
  {}

  /** Create a 3-D grid point. */
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

/** Universal Transverse Mercator (UTM) pose */
class UTMPose
{
 public:

  /** Null constructor. Makes a 2D, invalid pose object. */
  UTMPose():
    position(),
    orientation()
  {}

  /** Copy constructor. */
  UTMPose(const UTMPose &that):
    position(that.position),
    orientation(that.orientation)
  {}
  
  /** Create from a WGS 84 geodetic pose. */
  UTMPose(const geographic_msgs::GeoPose &pose):
    position(pose.position),
    orientation(pose.orientation)
  {}

  /** Create from a UTMPoint and a quaternion. */
  UTMPose(UTMPoint pt,
          const geometry_msgs::Quaternion &q):
    position(pt),
    orientation(q)
  {}

  /** Create from a WGS 84 geodetic point and a quaternion. */
  UTMPose(const geographic_msgs::GeoPoint &pt,
          const geometry_msgs::Quaternion &q):
    position(pt),
    orientation(q)
  {}

  // data members
  UTMPoint position;
  geometry_msgs::Quaternion orientation;

}; // class UTMPose

// conversion functions
void convert(const UTMPoint &from, geographic_msgs::GeoPoint &to);
void convert(const UTMPose &from, geographic_msgs::GeoPose &to);
void convert(const geographic_msgs::GeoPoint &from, UTMPoint &to);
void convert(const geographic_msgs::GeoPose &from, UTMPose &to);

/** @return true if no altitude specified. */
static inline bool is2D(const UTMPoint &pt)
{
  // true if altitude is a NaN
  return (pt.altitude != pt.altitude);
}

/** @return true if no altitude specified. */
static inline bool is2D(const UTMPose &pose)
{
  // true if position has no altitude
  return is2D(pose.position);
}

bool isValid(const UTMPoint &pt);
bool isValid(const UTMPose &pose);

/** normalize UTM point
 *
 *  @param pt point to be normalized
 *
 *  Ensures the point is within its canonical zone and band.
 */
static inline void normalize(UTMPoint &pt)
{
  geographic_msgs::GeoPoint ll;
  convert(pt, ll);
  normalize(ll);
  convert(ll, pt);
}

}; // namespace geodesy

#endif // _UTMPOINT_H_
