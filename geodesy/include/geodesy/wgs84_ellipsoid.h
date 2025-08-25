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

#ifndef GEODESY__WGS84_ELLIPSOID_H_
#define GEODESY__WGS84_ELLIPSOID_H_

#include "geodesy/ellipsoid.h"

namespace geodesy
{

namespace wgs84
{


/// Paremeters defining the WGS 84 ellipsoid.
struct EllipsoidParameters
{
  /// Semi-major axis
  static constexpr double a = 6378137.0;

  /// Minor axis
  static constexpr double b = 6356752.314245;

  /// Flattening
  static constexpr double f = 1.0 / 298.257223563;

  /// Angular velocity of the Earth in radians per second
  static constexpr double w = 7292115e-11;

  static constexpr double e2 = 1.0 - (b * b) / (a * a);
};

using Ellipsoid = geodesy::Ellipsoid < EllipsoidParameters >;

}  // namespace wgs84

}  // namespace geodesy

#endif  // GEODESY__WGS84_ELLIPSOID_H_
