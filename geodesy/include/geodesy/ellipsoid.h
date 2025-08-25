#ifndef GEODESY_ELLIPSOID_H
#define GEODESY_ELLIPSOID_H

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
      //U is 'reduced latitude'
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

}


#endif
