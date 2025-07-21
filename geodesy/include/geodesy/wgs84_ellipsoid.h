#ifndef GEODESY_WGS84_ELLIPSOID_H
#define GEODESY_WGS84_ELLIPSOID_H

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
  static constexpr double f = 1.0/298.257223563;

  /// Angular velocity of the Earth in radians per second
  static constexpr double w = 7292115e-11;

  static constexpr double e2 = 1.0-(b*b)/(a*a);
};

using Ellipsoid = geodesy::Ellipsoid<EllipsoidParameters>;

} // namespace wgs84

} // namespace geodesy

#endif
