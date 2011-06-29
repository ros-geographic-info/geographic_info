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

#ifndef _GEOTYPES_H_
#define _GEOTYPES_H_

#include <limits>
#include <ctype.h>
#include <ros/ros.h>
#include <geographic_msgs/GeoPoint.h>

/** @file

    @brief Geodetic coordinates data types

    @author Jack O'Quin
 */

namespace geodesy
{

/** Military Grid Reference System point. */
class GridPoint
{
 public:

  /** null constructor */
  GridPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(0),
    band(' ')
  {}

  /** copy constructor */
  GridPoint(const GridPoint &that):
    easting(that.easting),
    northing(that.northing),
    altitude(that.altitude),
    zone(that.zone),
    band(that.band)
  {}
  
  /** create from GeoPoint message */
  GridPoint(geographic_msgs::GeoPoint latlong);

  /** create flat grid point */
  GridPoint(double _easting, double _northing, uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(std::numeric_limits<double>::signaling_NaN()),
    zone(_zone),
    band(_band)
  {}

  /** create 3-D grid point */
  GridPoint(double _easting, double _northing, double _altitude,
            uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(_altitude),
    zone(_zone),
    band(_band)
  {}

  /** convert grid point to GeoPoint message */
  geographic_msgs::GeoPoint toGeoPoint();

  /** return true if this point has no altitude */
  bool isFlat(void)
  {
    // true if altitude is a NaN
    return (altitude != altitude);
  }

  /** return true if this point has a valid value */
  bool isValid(void)
  {
    if (zone < 1 || zone > 60)
      return false;

    if (!isupper(band) || band == 'I' || band == 'O')
      return false;

    // the Universal Polar Stereographic bands are not currently supported
    if (band < 'C' || band > 'X')
      return false;

    return true;
  }

  // data members
  double easting;           ///< easting within zone [meters]
  double northing;          ///< northing within zone [meters] 
  double altitude;          ///< altitude [meters], NaN if unspecified
  uint8_t zone;             ///< UTM longitude zone number
  char   band;              ///< MGRS latitude band letter

}; // class GridPoint

}; // namespace geodesy

#endif // _GEOTYPES_H_
