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

#ifndef GEODESY__ELLIPSOID_H_
#define GEODESY__ELLIPSOID_H_

#include <cmath>

namespace geodesy
{
  /// Static ellipsoid class for geodetic calculations
template < typename EllipsoidParameters >
class Ellipsoid
{
public:
  using Parameters = EllipsoidParameters;

  /// Meridional radius of curvature.
  /// Radius of curvature in north-south direction.
  /// @param latitude Latitude in radians.
  static double M(double latitude)
  {
    return Parameters::a * (1.0 - Parameters::e2) /
            pow((1.0 - Parameters::e2) * pow(sin(latitude), 2.0), 3.0 / 2.0);
  }

  /// Transverse radius of curvature.
  /// Radius of curvature in east-west direction.
  /// @param latitude Latitude in radians.
  static double N(double latitude)
  {
    if(Parameters::e2 == 0.0) {
      return Parameters::a;
    }
    return Parameters::a / sqrt(1 - Parameters::e2 * pow(sin(latitude), 2.0));
  }

  /// Calculate angle of longitude covered by distance in meters at given latitude in radians.
  /// From https://en.wikipedia.org/wiki/Longitude#Length_of_a_degree_of_longitude
  /// delta 1 long = (pi/180)a*cos(B) where tan(B) = (b/a)tan(phi) where B is reduced latitude
  static inline double longitudinal_span(double latitude, double distance)
  {
    // U is 'reduced latitude'
    double tanU1 = (1.0 - Parameters::f) * tan(latitude);
    double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
    return distance / (Parameters::a * cosU1);
  }

  /// Calculates approximate angle of latitude covered by distance in meters
  /// along longitudinal lines at given latitude.
  /// https://en.wikipedia.org/wiki/Latitude#Length_of_a_degree_of_latitude
  /// The length of a small meridian arc is given by:
  /// delta m(phi) = M(phi)*delta phi = a(1-e2)((1-e2*sin(phi)^2)^(-3/2)) *delta phi
  static inline double latitudinal_span(double latitude, double distance)
  {
    return distance * pow(1.0 - Parameters::e2 * pow(sin(latitude), 2),
      3.0 / 2.0) / (Parameters::a * (1 - Parameters::e2));
  }
};

}  // namespace geodesy

#endif  // GEODESY__ELLIPSOID_H_
