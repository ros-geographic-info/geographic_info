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

#include <gtest/gtest.h>
#include "geodesy/geodesics.h"


// From Vincenty's paper:
// First example is on Bessel, remaining are on international ellipsoid
// Bessel: a = 6377397.155m 1/f = r = 299.1528128
// International: a = 6378388.0m 1/f = r = 297
//
// Table II
//    phi1            alpha1          s              phi2             L
// a) 55 45 00.00000, 96 36 08.79960, 14110526.170m, -33 26 00.00000, 108 13 00.00000
// b) 37 19 54.95367, 95 27 59.63089,  4085966.703m,  26 07 42.83946,  41 28 35.50729
// c) 35 16 11.24862, 15 44 23.74850,  8084823.839m,  67 22 14.77638, 137 47 28.31435
// d)  1 00 00.00000, 89 00 00.00000, 19960000.000m,  -0 59 53.83076, 179 17 48.02997
// e)  1 00 00.00000,  4 59 59.99995, 19780006.558m,   1 01 15.18952, 179 46 17.84244

double dms_to_deg(float degrees, int minutes, double seconds)
{
  int sign = (degrees < 0 ||
    (degrees == 0 && (minutes < 0 || (minutes == 0 && seconds < 0)))) ? -1 : 1;
  return sign * (std::abs(degrees) + (std::abs(minutes) / 60.0) + (std::abs(seconds) / 3600.0));
}

struct BesselParameters
{
  static constexpr double a = 6377397.155;
  static constexpr double f = 1.0 / 299.1528128;
  static constexpr double b = a * (1 - f);
  static constexpr double e2 = 1.0 - (b * b) / (a * a);
};

struct InternationalParameters
{
  static constexpr double a = 6378388.0;
  static constexpr double f = 1.0 / 297.0;
  static constexpr double b = a * (1 - f);
  static constexpr double e2 = 1.0 - (b * b) / (a * a);
};

template<typename EllipsoidParameters>
static void test_paper_example(
  double phi1, double alpha1, double s,
  double phi2, double L)
{
  geographic_msgs::msg::GeoPoint p1;
  p1.latitude = phi1;
  p1.longitude = 0.0;
  p1.altitude = 0.0;

  geographic_msgs::msg::GeoPoint p2;
  p2.latitude = phi2;
  p2.longitude = L;
  p2.altitude = 0.0;

  alpha1 = alpha1 * M_PI / 180.0;  // Convert to radians

  auto p2_direct = geodesy::direct<EllipsoidParameters>(p1, alpha1, s);
  EXPECT_NEAR(p2_direct.latitude, p2.latitude, 0.00000001);
  EXPECT_NEAR(p2_direct.longitude, p2.longitude, 0.00000001);

  auto ad = geodesy::inverse<EllipsoidParameters>(p1, p2);
  EXPECT_NEAR(ad.azimuth, alpha1, 0.0000001);
  EXPECT_NEAR(ad.distance, s, 0.001);
}

TEST(Geodesics, paper_a)
{
  test_paper_example<BesselParameters>(
    dms_to_deg(55, 45, 0.0), dms_to_deg(96, 36, 8.79960),
    14110526.170, dms_to_deg(-33, 26, 0.0), dms_to_deg(108, 13, 0.0));
}

TEST(Geodesics, paper_b)
{
  test_paper_example<InternationalParameters>(
    dms_to_deg(37, 19, 54.95367), dms_to_deg(95, 27, 59.63089),
    4085966.703, dms_to_deg(26, 7, 42.83946), dms_to_deg(41, 28, 35.50729));
}

TEST(Geodesics, paper_c)
{
  test_paper_example<InternationalParameters>(
    dms_to_deg(35, 16, 11.24862), dms_to_deg(15, 44, 23.74850),
    8084823.839, dms_to_deg(67, 22, 14.77638), dms_to_deg(137, 47, 28.31435));
}

TEST(Geodesics, paper_d)
{
  test_paper_example<InternationalParameters>(
    dms_to_deg(1, 0, 0.0), dms_to_deg(89, 0, 0.0),
    19960000.0, dms_to_deg(0, -59, 53.83076), dms_to_deg(179, 17, 48.02997));
}

TEST(Geodesics, paper_e)
{
  test_paper_example<InternationalParameters>(
    dms_to_deg(1, 0, 0.0), dms_to_deg(4, 59, 59.99995),
    19780006.558, dms_to_deg(1, 1, 15.18952), dms_to_deg(179, 46, 17.84244));
}


// From:
// Vincenty, Thaddeus (August 1975b). Geodetic inverse solution between antipodal
// points (Technical report). DMAAC Geodetic Survey Squadron. doi:10.5281/zenodo.32999
// https://geographiclib.sourceforge.io/geodesic-papers/vincenty75b.pdf
//
// TABLE 1. Antipodal Solutions
//
// phi1     41 41 45.88000,   0 00 00.00000,  30 00 00.00000,  60 00 00.00000
// phi2    -41 41 46.20000,   0 00 00.00000, -30 00 00.00000, -59 59 00.00000
// L       179 59 59.44000, 179 41 49.78063, 179 40 00.00000, 179 50 00.00000
// alpha1, 179 58 49.16300,  30 00 00.000,    39 24 51.80600,  29 11 51.070
// alpha2,   0 01 10.83800, 150 00 00.000,   140 35 08.19400, 150 49 06.868
// s         20004566.7228, 19996147.4169,   19994364.6069, 20000433.9629
//
// TABLE 2. Back Side Solutions
//
// phi1     41 41 45.88000, 0 00 00.00000, 30 00 00.00000, 60 00 00.00000
// phi2     41 41 46.20000, 0 00 00.00000, 30 00 00.00000, 59 59 00.00000
// L         0.00 00.56000, 0 18 10.21937,  0 20 00.00000, 0 10 00.00000
// alpha1    180 00 35.423, 194 28 47.448, 198 30 47.488, 344 56 31.727
// alpha2    180 00 35.423, 194 28 47.448, 198 30 47.488, 344 56 59.622
// s         40009143.3208, 40004938.2722, 40004046.7114, 40006087.0024

template<typename EllipsoidParameters>
static void test_antipodal_paper_example(
  double phi1, double alpha1, double s,
  double phi2, double L)
{
  geographic_msgs::msg::GeoPoint p1;
  p1.latitude = phi1;
  p1.longitude = 0.0;
  p1.altitude = 0.0;

  geographic_msgs::msg::GeoPoint p2;
  p2.latitude = phi2;
  p2.longitude = L;
  p2.altitude = 0.0;

  alpha1 = alpha1 * M_PI / 180.0;  // Convert to radians

  auto p2_direct = geodesy::direct<EllipsoidParameters>(p1, alpha1, s);
  EXPECT_NEAR(p2_direct.latitude, p2.latitude, 0.00000001);
  EXPECT_NEAR(p2_direct.longitude, p2.longitude, 0.00000001);

  EXPECT_THROW(
  {
    auto ad = geodesy::inverse<EllipsoidParameters>(p1, p2);
    EXPECT_NEAR(ad.azimuth, alpha1, 0.0000001);
    EXPECT_NEAR(ad.distance, s, 0.001);
    },
    std::runtime_error
  );
}


TEST(Geodesics, antipodal_a)
{
  test_antipodal_paper_example<InternationalParameters>(
    dms_to_deg(41, 41, 45.88000), dms_to_deg(179, 58, 49.16300),
    20004566.7228, dms_to_deg(-41, 41, 46.20000), dms_to_deg(179, 59, 59.44000));
}

TEST(Geodesics, antipodal_b)
{
  test_antipodal_paper_example<InternationalParameters>(
    dms_to_deg(0, 0, 0.0), dms_to_deg(30, 0, 0.0),
    19996147.4169, dms_to_deg(0, 0, 0.0000), dms_to_deg(179, 41, 49.78063));
}

TEST(Geodesics, antipodal_c)
{
  test_antipodal_paper_example<InternationalParameters>(
    dms_to_deg(30, 0, 0.0), dms_to_deg(39, 24, 51.80600),
    19994364.6069, dms_to_deg(-30, 0, 0.0), dms_to_deg(179, 40, 0.00000));
}

TEST(Geodesics, antipodal_d)
{
  test_antipodal_paper_example<InternationalParameters>(
    dms_to_deg(60, 0, 0.0), dms_to_deg(29, 11, 51.070),
    20000433.9629, dms_to_deg(-59, 59, 0.0), dms_to_deg(179, 50, 0.00000));
}

template<typename EllipsoidParameters>
static void test_back_side_paper_example(
  double phi1, double alpha1, double s,
  double phi2, double L)
{
  geographic_msgs::msg::GeoPoint p1;
  p1.latitude = phi1;
  p1.longitude = 0.0;
  p1.altitude = 0.0;

  geographic_msgs::msg::GeoPoint p2;
  p2.latitude = phi2;
  p2.longitude = L;
  p2.altitude = 0.0;

  alpha1 = alpha1 * M_PI / 180.0;  // Convert to radians

  auto p2_direct = geodesy::direct<EllipsoidParameters>(p1, alpha1, s);
  EXPECT_NEAR(p2_direct.latitude, p2.latitude, 0.00000001);
  EXPECT_NEAR(p2_direct.longitude, p2.longitude, 0.00000001);
}


TEST(Geodesics, back_side_a)
{
  test_back_side_paper_example<InternationalParameters>(
    dms_to_deg(41, 41, 45.88000), dms_to_deg(180, 0, 35.423),
    40009143.3208, dms_to_deg(41, 41, 46.20000), dms_to_deg(0, 0, 0.56000));
}

TEST(Geodesics, back_side_b)
{
  test_back_side_paper_example<InternationalParameters>(
    dms_to_deg(0, 0, 0.0), dms_to_deg(194, 28, 47.448),
    40004938.2722, dms_to_deg(0, 0, 0.00000), dms_to_deg(0, 18, 10.21937));
}

TEST(Geodesics, back_side_c)
{
  test_back_side_paper_example<InternationalParameters>(
    dms_to_deg(30, 0, 0.0), dms_to_deg(198, 30, 47.488),
    40004046.7114, dms_to_deg(30, 0, 0.0), dms_to_deg(0, 20, 0.00000));
}

TEST(Geodesics, back_side_d)
{
  test_back_side_paper_example<InternationalParameters>(
    dms_to_deg(60, 0, 0.0), dms_to_deg(344, 56, 31.727),
    40006087.0024, dms_to_deg(59, 59, 0.0), dms_to_deg(0, 10, 0.00000));
}


// echo 43.071873 -70.712142 65 100 | GeodSolve
// 43.07225341 -70.71102922 65.00075994

TEST(Geodesics, direct_100m)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 100.0);

  EXPECT_NEAR(end.latitude, 43.07225341, 0.000001);
  EXPECT_NEAR(end.longitude, -70.71102922, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (65.00075994 + 180.0) * M_PI / 180.0, 100.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}


// echo 43.071873 -70.712142 65 1000 | GeodSolve
// 43.07567660 -70.70101358 65.00760004

TEST(Geodesics, direct_1000m)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 1000.0);

  EXPECT_NEAR(end.latitude, 43.07567660, 0.000001);
  EXPECT_NEAR(end.longitude, -70.70101358, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (65.00760004 + 180.0) * M_PI / 180.0, 1000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// echo 43.071873 -70.712142 65 10000 | GeodSolve
// 43.10986018 -70.60079586 65.07606696

TEST(Geodesics, direct_10km)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 10000.0);

  EXPECT_NEAR(end.latitude, 43.10986018, 0.000001);
  EXPECT_NEAR(end.longitude, -70.60079586, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (65.07606696 + 180.0) * M_PI / 180.0, 10000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// echo 43.071873 -70.712142 65 100000 | GeodSolve
// 43.44681882 -69.59249410 65.76731383

TEST(Geodesics, direct_100km)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 100000.0);

  EXPECT_NEAR(end.latitude, 43.44681882, 0.000001);
  EXPECT_NEAR(end.longitude, -69.59249410, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (65.76731383 + 180.0) * M_PI / 180.0, 100000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// echo 43.071873 -70.712142 65 1000000 | GeodSolve
// 46.28803167 -58.91595973 73.31258685

TEST(Geodesics, direct_1000km)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 1000000.0);

  EXPECT_NEAR(end.latitude, 46.28803167, 0.000001);
  EXPECT_NEAR(end.longitude, -58.91595973, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (73.31258685 + 180.0) * M_PI / 180.0, 1000000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// echo 43.071873 -70.712142 65 10000000 | GeodSolve
// 18.06209909 36.72816601 135.79422660

TEST(Geodesics, direct_10_000km)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 10000000.0);

  EXPECT_NEAR(end.latitude, 18.06209909, 0.000001);
  EXPECT_NEAR(end.longitude, 36.72816601, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (135.79422660 + 180.0) * M_PI / 180.0,
    10000000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// echo 43.071873 -70.712142 65 20000000 | GeodSolve
// -43.11302321 109.00850466 114.91758277

TEST(Geodesics, direct_20_000km)
{
  geographic_msgs::msg::GeoPoint start;
  start.latitude = 43.071873;
  start.longitude = -70.712142;
  start.altitude = 0.0;

  auto end = geodesy::wgs84::direct(start, 65.0 * M_PI / 180.0, 20000000.0);

  EXPECT_NEAR(end.latitude, -43.11302321, 0.000001);
  EXPECT_NEAR(end.longitude, 109.00850466, 0.000001);
  EXPECT_NEAR(end.altitude, 0.0, 0.000001);

  auto back_at_start = geodesy::wgs84::direct(end, (114.91758277 + 180.0) * M_PI / 180.0,
    20000000.0);
  EXPECT_NEAR(back_at_start.latitude, 43.071873, 0.00001);
  EXPECT_NEAR(back_at_start.longitude, -70.712142, 0.000001);
  EXPECT_NEAR(back_at_start.altitude, 0.0, 0.000001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
