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

#ifndef GEODESY__GEODESICS_H_
#define GEODESY__GEODESICS_H_


#include <cmath>

#include "geodesy/wgs84.h"
#include "geodesy/wgs84_ellipsoid.h"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace geodesy
{

// Implementation of direct and inverse geodesic calculations using Vincenty's formulae.
//
// Vincenty, Thaddeus (April 1975a). "Direct and Inverse Solutions of Geodesics on the
// Ellipsoid with application of nested equations". Survey Review. XXIII (176): 88–93.
// https://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf
//
// Vincenty, Thaddeus (August 1975b). Geodetic inverse solution between antipodal
// points (Technical report). DMAAC Geodetic Survey Squadron. doi:10.5281/zenodo.32999
// https://geographiclib.sourceforge.io/geodesic-papers/vincenty75b.pdf


/// Helper structure to efficiently return azimuth and distance.
struct AzimuthDistance
{
  double azimuth;   ///< Azimuth in radians, clockwise from north.
  double distance;  ///< Distance in meters.
};

/// Calculates the postion p2 from azimuth and distance from p1 on the specified ellipsoid.
/// @param p1 starting point in degrees. Altitude must be zero.
/// @param azimuth clockwise angle in radians relative to north.
/// @param distance distance in meters.
template < typename EllipsoidParameters >
static geographic_msgs::msg::GeoPoint direct(
  const geographic_msgs::msg::GeoPoint & p1,
  double azimuth, double distance)
{
  // Implemented using Vincenty's formulae.
  // https://en.wikipedia.org/wiki/Vincenty%27s_formulae

  if(!is2D(p1) && p1.altitude != 0.0) {
    throw std::invalid_argument(
      "Altitude must be zero or not specified for direct geodesic calculations.");
  }

  // Convert to radians
  double phi1 = p1.latitude * M_PI / 180.0;
  double alpha1 = azimuth;

  // Vincenty uses an epsilon of 1e-12.
  double epsilon = 1e-12;

  // U is 'reduced latitude'
  double tanU1 = (1.0 - EllipsoidParameters::f) * tan(phi1);
  double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
  double sinU1 = tanU1 * cosU1;

  double cosAlpha1 = cos(alpha1);
  double sinAlpha1 = sin(alpha1);

  // angular distance on sphere from equator to P1 along geodesic
  double sigma1 = atan2(tanU1, cosAlpha1);

  // Eq (2)
  double sinAlpha = cosU1 * sinAlpha1;
  double cos2Alpha = 1.0 - sinAlpha * sinAlpha;

  double a = EllipsoidParameters::a;
  double b = EllipsoidParameters::b;
  double f = EllipsoidParameters::f;

  double u2 = cos2Alpha * (a * a - b * b) / (b * b);

  double k1 = (sqrt(1.0 + u2) - 1.0) / (sqrt(1.0 + u2) + 1.0);

  // Using Vincenty's 1976 modification
  // Replaces Eq (3)
  double A = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
  // Replaces Eq (4)
  double B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);

  // Angular distance from P1 to P2 on the sphere
  double sigma = distance / (b * A);
  double last_sigma;
  double cos2Sigmam;

  uint16_t iteration_count = 0;
  while (true) {
    iteration_count++;
    if (iteration_count > 10) {
      throw std::runtime_error("Geodesic direct formula did not converge after 1000 iterations.");
    }
    // cosine of Eq (5)
    cos2Sigmam = cos(2.0 * sigma1 + sigma);

    double sinSigma = sin(sigma);

    double cosSigma = cos(sigma);

    // Eq (6)
    double deltaSigma = B * sinSigma *
      (cos2Sigmam + 0.25 * B * (cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam) -
      (B / 6.0) * cos2Sigmam * (-3.0 + 4.0 * sinSigma * sinSigma) *
      (-3.0 + 4.0 * cos2Sigmam * cos2Sigmam)
      )
      );

    last_sigma = sigma;

    // Eq (7)
    sigma = (distance / (b * A)) + deltaSigma;

    if (fabs(last_sigma - sigma) <= epsilon) {
      break;
    }
  }
  cos2Sigmam = cos(2.0 * sigma1 + sigma);

  // arctangent of Eq (8)
  double phi2 = atan2(sinU1 * cos(sigma) + cosU1 * sin(sigma) * cosAlpha1,
      (1 - f) * sqrt(
        sinAlpha * sinAlpha + pow(sinU1 * sin(sigma) - cosU1 * cos(sigma) * cosAlpha1, 2)
  ));

  // arctangent of Eq (9)
  double l = atan2(sin(sigma) * sinAlpha1, cosU1 * cos(sigma) - sinU1 * sin(sigma) * cosAlpha1);

  // Eq (10)
  double C = (f / 16.0) * cos2Alpha * (4.0 + f * (4.0 - 3.0 * cos2Alpha));

  // Eq (11)
  double L = l - (1.0 - C) * f * sinAlpha *
    (sigma + C * sin(sigma) *
    (cos2Sigmam + C * cos(sigma) * (-1 + 2.0 * cos2Sigmam * cos2Sigmam)));

  if(L < -M_PI) {
    L += 2 * M_PI;
  } else if(L > M_PI) {
    L -= 2 * M_PI;
  }

  // Convert back to degrees
  geographic_msgs::msg::GeoPoint result;
  result.latitude = phi2 * 180.0 / M_PI;
  result.longitude = p1.longitude + (L * 180.0 / M_PI);
  result.altitude = p1.altitude;  // Keep the same altitude

  return result;
}

template < typename EllipsoidParameters >
static geographic_msgs::msg::GeoPoint direct(
  const geographic_msgs::msg::GeoPoint & p1,
  AzimuthDistance azimuth_distance)
{
  return direct < EllipsoidParameters > (p1, azimuth_distance.azimuth, azimuth_distance.distance);
}

/// Calculates the postion p2 from a direction vector from p1 on the specified ellipsoid.
/// The direction vector is expected to have a zero z component.
/// @param p1 starting point in degrees. Altitude must be zero.
/// @param direction direction vector with x (east) and y (north) components representing the
///   azimuth and distance.
template < typename EllipsoidParameters >
static geographic_msgs::msg::GeoPoint direct(
  const geographic_msgs::msg::GeoPoint & p1,
  const geometry_msgs::msg::Vector3 & direction)
{
  if(direction.z != 0.0 || !std::isnan(direction.z)) {
    throw std::invalid_argument(
      "Direction vector must have z component zero or not specified for direct geodesic "
      "calculations.");
  }

  /// Convert azimuth from ccw angle relative to x (east) to clockwise angle relative to y (north).
  /// (REP103 to compass heading)
  double azimuth = (M_PI / 2.0) - atan2(direction.y, direction.x);
  double distance = sqrt(direction.x * direction.x + direction.y * direction.y);

  return direct < EllipsoidParameters > (p1, azimuth, distance);
}


/// Calculates the azimuth and distance from p1 to p2 on the specified ellipsoid.
/// @param p1: Position P1 in degrees
/// @param p2: Position P2 in degrees
/// @return: azimuth in radians, distance in meters
template < typename EllipsoidParameters >
static AzimuthDistance inverse(
  const geographic_msgs::msg::GeoPoint & p1,
  const geographic_msgs::msg::GeoPoint & p2)
{
  // Implemented using Vincenty's formulae.
  // https://en.wikipedia.org/wiki/Vincenty%27s_formulae

  if(!is2D(p1) && p1.altitude != 0.0 || !is2D(p2) && p2.altitude != 0.0) {
    throw std::invalid_argument(
      "Altitude must be zero or not specified for inverse geodesic calculations.");
  }

  if(p1.latitude == p2.latitude && p1.longitude == p2.longitude) {
    return {0.0, 0.0};  // Same point, no azimuth or distance
  }

  double a = EllipsoidParameters::a;
  double b = EllipsoidParameters::b;
  double f = EllipsoidParameters::f;

  double epsilon = 1e-12;

  double phi1 = p1.latitude * M_PI / 180.0;
  double phi2 = p2.latitude * M_PI / 180.0;

  // difference in longitude
  double L = (p2.longitude - p1.longitude) * M_PI / 180.0;

  // reduced latitudes
  double U1 = atan((1.0 - f) * tan(phi1));
  double U2 = atan((1.0 - f) * tan(phi2));

  double cosU1 = cos(U1);
  double cosU2 = cos(U2);
  double sinU1 = sin(U1);
  double sinU2 = sin(U2);

  // Eq (13), difference in longitude on auxiliary sphere
  double lambda = L;

  double sinSigma;
  double cosSigma;

  // angular distance on the sphere
  double sigma;

  double cos2Alpha;
  double sinAlpha;
  double cos2Sigmam;
  double alpha1 = std::nan("");
  double A = std::nan("");
  double B = std::nan("");

  double last_lambda = std::nan("");
  double cosLambda;
  double sinLambda;

  uint16_t iteration_count = 0;

  while (true) {
    iteration_count++;
    if (iteration_count > 200) {
      throw std::runtime_error("Geodesic inverse formula did not converge after 200 iterations.");
    }


    cosLambda = cos(lambda);
    sinLambda = sin(lambda);

    // Square root of Eq (14)
    sinSigma = sqrt((pow((cosU2 * sinLambda),
      2)) + pow((cosU1 * sinU2 - sinU1 * cosU2 * cosLambda), 2));

    // Eq (15)
    cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;

    // Arctangent of Eq (16), angular distance P1 to P2 on the sphere
    sigma = atan2(sinSigma, cosSigma);

    // Eq (17)
    sinAlpha = (cosU1 * cosU2 * sinLambda) / sinSigma;

    cos2Alpha = 1 - sinAlpha * sinAlpha;

    // Eq (10)
    double C = (f / 16.0) * cos2Alpha * (4.0 + f * (4.0 - 3.0 * cos2Alpha));

    if (cos2Alpha == 0) {
      cos2Sigmam = 0;
    } else {
      // Eq (18)
      cos2Sigmam = cosSigma - ((2.0 * sinU1 * sinU2) / cos2Alpha);
    }


    if (!std::isnan(last_lambda) && fabs(last_lambda - lambda) <= epsilon) {
      break;
    }
    last_lambda = lambda;

    // reverse of Eq (11)
    lambda = L + (1.0 - C) * f * sinAlpha *
      (sigma + C * sinSigma *
      (cos2Sigmam + C * cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam)));

    if(fabs(lambda) > M_PI) {
      throw std::runtime_error(
        "Geodesic inverse formula failed: lambda > π (Antipodal case not handled)");
    }
  }

  double u2 = cos2Alpha * (a * a - b * b) / (b * b);
  // Using Vincenty's 1976 modification
  double k1 = (sqrt(1.0 + u2) - 1.0) / (sqrt(1.0 + u2) + 1.0);
  A = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
  B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);

  // Arctangent of Eq (20)
  alpha1 = atan2(cosU2 * sinLambda, cosU1 * sinU2 - sinU1 * cosU2 * cosLambda);

  // 1975a Eq (6)
  double deltaSigma = B * sinSigma *
    (cos2Sigmam + 0.25 * B *
    (cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam) - (B / 6.0) * cos2Sigmam *
    (-3.0 + 4.0 * sinSigma * sinSigma) * (-3.0 + 4.0 * cos2Sigmam * cos2Sigmam)));

  // 1975a Eq (19)
  double s = b * A * (sigma - deltaSigma);

  double azimuth = fmod((alpha1 + 2 * M_PI), (2 * M_PI));  // Normalize to [0, 2π)

  AzimuthDistance result;
  result.azimuth = azimuth;  // Azimuth in radians, clockwise from north
  result.distance = s;  // Distance in meters
  return result;
}

/// Calculates the inverse direction vector from p1 to p2 on the specified ellipsoid.
/// The result is a vector with x and y components representing the east and north
/// components of the direction.
template < typename EllipsoidParameters >
static geometry_msgs::msg::Vector3 inverse_vector(
  const geographic_msgs::msg::GeoPoint & p1,
  const geographic_msgs::msg::GeoPoint & p2)
{
  AzimuthDistance ad = inverse < EllipsoidParameters > (p1, p2);
  double azimuth = (M_PI / 2.0) - ad.azimuth;  // Convert azimuth to east-north coordinate system
  geometry_msgs::msg::Vector3 result;
  result.x = ad.distance * cos(azimuth);  // East component
  result.y = ad.distance * sin(azimuth);  // North component
  result.z = 0.0;  // Z component is zero for horizontal direction
  return result;
}

namespace wgs84
{
  /// Calculates the postion p2 from azimuth and distance from p1 on the WGS84 ellipsoid.
  /// @param p1 starting point (altitude must be zero)
  /// @param azimuth clockwise angle in radians relative to north.
  /// @param distance distance in meters.
  static inline geographic_msgs::msg::GeoPoint direct(
    const geographic_msgs::msg::GeoPoint & p1,
    double azimuth, double distance)
  {
    return geodesy::direct < wgs84::Ellipsoid::Parameters > (p1, azimuth, distance);
  }

  /// Calculates the azimuth and distance from p1 to p2 on the WGS84 ellipsoid.
  /// @param p1: Position P1 in radians (altitude must be zero)
  /// @param p2: Position P2 in radians (altitude must be zero)
  /// @return: azimuth in radians, distance in meters
  static inline AzimuthDistance inverse(
    const geographic_msgs::msg::GeoPoint & p1,
    const geographic_msgs::msg::GeoPoint & p2)
  {
    return geodesy::inverse < wgs84::Ellipsoid::Parameters > (p1, p2);
  }

  /// Calculates the inverse direction vector from p1 to p2 on the specified ellipsoid.
  /// The result is a vector with x and y components representing the east and north
  /// components of the direction.

  static geometry_msgs::msg::Vector3 inverse_vector(
    const geographic_msgs::msg::GeoPoint & p1,
    const geographic_msgs::msg::GeoPoint & p2)
  {
    return geodesy::inverse_vector < wgs84::Ellipsoid::Parameters > (p1, p2);
  }

}  // namespace wgs84

}  // namespace geodesy


#endif  // GEODESY__GEODESICS_H_
