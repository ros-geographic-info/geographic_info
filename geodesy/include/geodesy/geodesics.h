#ifndef GEODESY_GEODESICS_H
#define GEODESY_GEODESICS_H


#include <cmath>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace geodesy
{

  /// Helper structure to efficiently return azimuth and distance.
  struct AzimuthDistance
  {
    double azimuth; ///< Azimuth in radians, clockwise from north.
    double distance; ///< Distance in meters.
  };

  /// Calculates the postion p2 from azimuth and distance from p1 on the specified ellipsoid.
  /// @param p1 starting point
  /// @param azimuth clockwise angle in radians relative to north.
  /// @param distance distance in meters.
  template <typename EllipsoidParameters>
  static geographic_msgs::msg::GeoPoint direct(
      const geographic_msgs::msg::GeoPoint &p1,
      double azimuth, double distance)
  {
    if(p1.altitude != 0.0 || !std::isnan(p1.altitude))
    {
      throw std::invalid_argument("Altitude must be zero or not specified for direct geodesic calculations.");
    }

    // Convert to radians
    double phi1 = p1.latitude * M_PI / 180.0;
    double alpha1 = p1.longitude * M_PI / 180.0;

    double epsilon = 1e-12;

    //U is 'reduced latitude'
    double tanU1 = (1.0 - EllipsoidParameters::f) * tan(phi1);
    double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
    double sinU1 = tanU1 * cosU1;

    double cosAlpha1 = cos(alpha1);
    double sinAlpha1 = sin(alpha1);

    double sigma1 = atan2(tanU1, cosAlpha1); // angular distance on sphere from equator to P1 along geodesic
    double sinAlpha = cosU1 * sinAlpha1;
    double cos2Alpha = 1.0 - sinAlpha * sinAlpha;

    double a = EllipsoidParameters::a;
    double b = EllipsoidParameters::b;
    double f = EllipsoidParameters::f;

    double u2 = cos2Alpha * (a * a - b * b) / (b * b);

    double k1 = (sqrt(1.0 + u2) - 1.0) / (sqrt(1.0 + u2) + 1.0);
    double A = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
    double B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);

    double sigma = distance / (b * A);
    double last_sigma;
    double cos2Sigmam;

    while (true)
    {
      cos2Sigmam = cos(2.0 * sigma1 + sigma);
      double sinSigma = sin(sigma);
      double cosSigma = cos(sigma);

      double deltaSigma = B * sinSigma * (cos2Sigmam + 0.25 * B * (cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam) - (B / 6.0) * cos2Sigmam * (-3.0 + 4.0 * sinSigma * sinSigma) * (-3.0 + 4.0 * cos2Sigmam * cos2Sigmam)));
      last_sigma = sigma;
      sigma = (distance / (b * A)) + deltaSigma;
      if (fabs(last_sigma - sigma) <= epsilon)
        break;
    }
    cos2Sigmam = cos(2.0 * sigma1 + sigma);
    double phi2 = atan2(sinU1 * cos(sigma) + cosU1 * sin(sigma) * cosAlpha1, (1 - f) * sqrt(sinAlpha * sinAlpha + pow(sinU1 * sin(sigma) - cosU1 * cos(sigma) * cosAlpha1, 2)));
    double l = atan2(sin(sigma) * sinAlpha1, cosU1 * cos(sigma) - sinU1 * sin(sigma) * cosAlpha1);
    double C = (f / 16.0) * cos2Alpha * (4.0 + f * (4.0 - 3.0 * cos2Alpha));
    double L = l - (1.0 - C) * f * sinAlpha * (sigma + C * sin(sigma) * (cos2Sigmam + C * cos(sigma) * (-1 + 2.0 * cos2Sigmam * cos2Sigmam)));

    // Convert back to degrees
    geographic_msgs::msg::GeoPoint result;
    result.latitude = phi2 * 180.0 / M_PI;
    result.longitude = (alpha1 + L) * 180.0 / M_PI;
    result.altitude = p1.altitude; // Keep the same altitude

    return result;
  }

  template <typename EllipsoidParameters>
  static geographic_msgs::msg::GeoPoint direct(
      const geographic_msgs::msg::GeoPoint &p1,
      AzimuthDistance azimuth_distance)
  {
    return direct<EllipsoidParameters>(p1, azimuth_distance.azimuth, azimuth_distance.distance);
  }

  /// Calculates the postion p2 from a direction vector from p1 on the specified ellipsoid.
  /// The direction vector is expected to have a zero z component.
  /// @param p1 starting point
  /// @param direction direction vector with x and y components representing the azimuth and distance.

  template <typename EllipsoidParameters>
  static geographic_msgs::msg::GeoPoint direct(
      const geographic_msgs::msg::GeoPoint &p1,
      const geometry_msgs::msg::Vector3 &direction)
  {
    if(direction.z != 0.0 || !std::isnan(direction.z))
    {
      throw std::invalid_argument("Direction vector must have z component zero or not specified for direct geodesic calculations.");
    }

    /// Convert azimuth from ccw angle relative to x (east) to clockwise angle relative to y (north). (REP103 to compass heading)
    double azimuth = (M_PI/2.0)-atan2(direction.y, direction.x);
    double distance = sqrt(direction.x * direction.x + direction.y * direction.y);

    return direct<EllipsoidParameters>(p1, azimuth, distance);
  }


  /// Calculates the azimuth and distance from p1 to p2 on the specified ellipsoid.
  /// @param p1: Position P1 in radians
  /// @param p2: Position P2 in radians
  /// @return: azimuth in radians, distance in meters
  template <typename EllipsoidParameters>
  static AzimuthDistance inverse(
    const geographic_msgs::msg::GeoPoint &p1,
    const geographic_msgs::msg::GeoPoint &p2)
  {
    if(p1.altitude != 0.0 || !std::isnan(p1.altitude) ||
       p2.altitude != 0.0 || !std::isnan(p2.altitude))
    {
      throw std::invalid_argument("Altitude must be zero or not specified for inverse geodesic calculations.");
    }

    if(p1.latitude == p2.latitude && p1.longitude == p2.longitude)
    {
      return {0.0, 0.0}; // Same point, no azimuth or distance
    }

    double a = EllipsoidParameters::a;
    double b = EllipsoidParameters::b;
    double f = EllipsoidParameters::f;

    double epsilon = 1e-12;

    double phi1 = p1.latitude * M_PI / 180.0;
    double phi2 = p2.latitude * M_PI / 180.0;

    double L = (p2.longitude - p1.longitude) * M_PI / 180.0;

    double U1 = atan((1.0 - f) * tan(phi1));
    double U2 = atan((1.0 - f) * tan(phi2));
    double cosU1 = cos(U1);
    double cosU2 = cos(U2);
    double sinU1 = sin(U1);
    double sinU2 = sin(U2);

    double l = L;
    double last_l = std::nan("");
    double cosl;
    double sinl;
    double sinSigma;
    double cosSigma;
    double sigma;
    double cos2Alpha;
    double cos2Sigmam;

    while (true)
    {
      cosl = cos(l);
      sinl = sin(l);

      sinSigma = sqrt((pow((cosU2 * sinl), 2)) + pow((cosU1 * sinU2 - sinU1 * cosU2 * cosl), 2));
      cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosl;
      sigma = atan2(sinSigma, cosSigma);
      double sinAlpha = (cosU1 * cosU2 * sinl) / sinSigma;

      cos2Alpha = 1 - sinAlpha * sinAlpha;
      if (cos2Alpha == 0)
        cos2Sigmam = 0;
      else
        cos2Sigmam = cosSigma - ((2.0 * sinU1 * sinU2) / cos2Alpha);

      if (!std::isnan(last_l) && fabs(last_l - l) <= epsilon)
        break;
      last_l = l;

      double C = (f / 16.0) * cos2Alpha * (4.0 + f * (4.0 - 3.0 * cos2Alpha));
      l = L + (1.0 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2Sigmam + C * cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam)));
    }

    double u2 = cos2Alpha * (a * a - b * b) / (b * b);
    double k1 = (sqrt(1.0 + u2) - 1.0) / (sqrt(1.0 + u2) + 1.0);
    double A = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
    double B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);
    double deltaSigma = B * sinSigma * (cos2Sigmam + 0.25 * B * (cosSigma * (-1.0 + 2.0 * cos2Sigmam * cos2Sigmam) - (B / 6.0) * cos2Sigmam * (-3.0 + 4.0 * sinSigma * sinSigma) * (-3.0 + 4.0 * cos2Sigmam * cos2Sigmam)));
    double s = b * A * (sigma - deltaSigma);
    double alpha1 = atan2(cosU2 * sinl, cosU1 * sinU2 - sinU1 * cosU2 * cosl);
    double azimuth = fmod((alpha1 + 2 * M_PI), (2 * M_PI)); // Normalize to [0, 2π)

    AzimuthDistance result;
    result.azimuth = azimuth; // Azimuth in radians, clockwise from north
    result.distance = s; // Distance in meters
    return result;
  }

  /// Calculates the inverse direction vector from p1 to p2 on the specified ellipsoid.
  /// The result is a vector with x and y components representing the east and north components of the direction.
  template <typename EllipsoidParameters>
  static geometry_msgs::msg::Vector3 inverse_vector(
    const geographic_msgs::msg::GeoPoint &p1,
    const geographic_msgs::msg::GeoPoint &p2)
  {
    AzimuthDistance ad = inverse<EllipsoidParameters>(p1, p2);
    double azimuth = (M_PI/2.0) - ad.azimuth; // Convert azimuth to east-north coordinate system
    geometry_msgs::msg::Vector3 result;
    result.x = ad.distance * cos(azimuth); // East component
    result.y = ad.distance * sin(azimuth); // North component
    result.z = 0.0; // Z component is zero for horizontal direction
    return result;
  }



}


#endif
