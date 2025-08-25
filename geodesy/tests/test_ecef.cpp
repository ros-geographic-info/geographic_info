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
#include <cmath>
#include "geodesy/ecef.h"

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test null constructor
TEST(ECEFPoint, nullConstructor)
{
  geodesy::ECEFPoint pt;

  EXPECT_EQ(std::isnan(pt.x), true);
  EXPECT_EQ(std::isnan(pt.y), true);
  EXPECT_EQ(std::isnan(pt.z), true);

  EXPECT_FALSE(geodesy::isValid(pt));
}

// From online converter:
// https://www.convertecef.com/index?x=1541491.429281025&y=-4404522.615551788&z=4333366.082882879
// UNH Pier: 43.072220, -70.711014
// 1541491.43, -4404522.62, 4333366.08

TEST(ECEFPoint, constructor)
{
  geodesy::ECEFPoint pt(1541491.43, -4404522.62, 4333366.08);

  EXPECT_EQ(pt.x, 1541491.43);
  EXPECT_EQ(pt.y, -4404522.62);
  EXPECT_EQ(pt.z, 4333366.08);
  EXPECT_TRUE(geodesy::isValid(pt));
}

TEST(ECEFPoint, copyConstructor)
{
  geodesy::ECEFPoint pt1(1541491.43, -4404522.62, 4333366.08);
  geodesy::ECEFPoint pt2(pt1);

  EXPECT_EQ(pt2.x, pt1.x);
  EXPECT_EQ(pt2.y, pt1.y);
  EXPECT_EQ(pt2.z, pt1.z);
  EXPECT_TRUE(geodesy::isValid(pt2));
}

TEST(ECEFPoint, fromGeoPoint)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 43.072220;
  geo_point.longitude = -70.711014;
  geo_point.altitude = 0.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 1541491.43, 0.01);
  EXPECT_NEAR(pt.y, -4404522.62, 0.01);
  EXPECT_NEAR(pt.z, 4333366.08, 0.01);
}

// https://www.convertecef.com/index?x=1541551.7557990744&y=-4404694.987261432&z=4333536.812800979
// ECEF:
// [ 1541551.76, -4404694.99, 4333536.81 ]
// LLA:
// 43.0722200, -70.7110140, 250.0 (degrees, degrees, meters HAE )

TEST(ECEFPoint, fromGeoPointAboveEllipsoid)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 43.072220;
  geo_point.longitude = -70.711014;
  geo_point.altitude = 250.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 1541551.76, 0.01);
  EXPECT_NEAR(pt.y, -4404694.99, 0.01);
  EXPECT_NEAR(pt.z, 4333536.81, 0.01);
}


// https://www.convertecef.com/index?x=1541370.776244926&y=-4404177.872132502&z=4333024.6230466785
// ECEF:
// [ 1541370.78, -4404177.87, 4333024.62 ]
// LLA:
// 43.0722200, -70.7110140, -500.0 (degrees, degrees, meters HAE )
TEST(ECEFPoint, fromGeoPointBelowEllipsoid)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 43.072220;
  geo_point.longitude = -70.711014;
  geo_point.altitude = -500.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 1541370.78, 0.01);
  EXPECT_NEAR(pt.y, -4404177.87, 0.01);
  EXPECT_NEAR(pt.z, 4333024.62, 0.01);
}


// https://www.convertecef.com/index?x=6378137.0&y=0.0&z=0.0
// ECEF:
// [ 6378137.0, 0.0, 0.0 ]
// LLA:
// 0.0000000, 0.0000000, 0.0 (degrees, degrees, meters HAE )


TEST(ECEFPoint, fromGeoPointAtZeroZeroZero)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 0.0;
  geo_point.longitude = 0.0;
  geo_point.altitude = 0.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 6378137.0, 0.01);  // ECEF X at equator
  EXPECT_NEAR(pt.y, 0.0, 0.01);        // ECEF Y at equator
  EXPECT_NEAR(pt.z, 0.0, 0.01);        // ECEF Z at equator
}

// https://www.convertecef.com/index?x=3.9186209248144716e-10&y=0.0&z=6356752.314245179
// ECEF:
// [ 0.0, 0.0, 6356752.31 ]
// LLA:
// 90.0000000, 0.0000000, -0.0 (degrees, degrees, meters HAE )


TEST(ECEFPoint, fromGeoPointAtNorthPole)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 90.0;  // North Pole
  geo_point.longitude = 0.0;
  geo_point.altitude = 0.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 0.0, 0.01);  // ECEF X at North Pole
  EXPECT_NEAR(pt.y, 0.0, 0.01);  // ECEF Y at North Pole
  EXPECT_NEAR(pt.z, 6356752.31, 0.01);  // ECEF Z at North Pole

  geographic_msgs::msg::GeoPoint geo_point2;
  geo_point2.latitude = 90.0;  // North Pole
  geo_point2.longitude = 45.0;
  geo_point2.altitude = 0.0;

  geodesy::ECEFPoint pt2(geo_point2);

  EXPECT_TRUE(geodesy::isValid(pt2));
  EXPECT_NEAR(pt2.x, pt.x, 0.01);  // ECEF X should be the same
  EXPECT_NEAR(pt2.y, pt.y, 0.01);  // ECEF Y should be the same
  EXPECT_NEAR(pt2.z, pt.z, 0.01);  // ECEF Z should be the same
}

TEST(ECEFPoint, fromGeoPointAtSouthPole)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = -90.0;  // South Pole
  geo_point.longitude = 0.0;
  geo_point.altitude = 0.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, 0.0, 0.01);  // ECEF X at South Pole
  EXPECT_NEAR(pt.y, 0.0, 0.01);  // ECEF Y at South Pole
  EXPECT_NEAR(pt.z, -6356752.31, 0.01);  // ECEF Z at South Pole
}

TEST(ECEFPoint, fromGeoPointAtInternationalDateLine)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = 0.0;  // Equator
  geo_point.longitude = 180.0;  // International Date Line
  geo_point.altitude = 0.0;

  geodesy::ECEFPoint pt(geo_point);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_NEAR(pt.x, -6378137.0, 0.01);  // ECEF X at IDL
  EXPECT_NEAR(pt.y, 0.0, 0.01);         // ECEF Y at IDL
  EXPECT_NEAR(pt.z, 0.0, 0.01);         // ECEF Z at IDL

  geographic_msgs::msg::GeoPoint geo_point2;
  geo_point2.latitude = 0.0;  // Equator
  geo_point2.longitude = -180.0;  // International Date Line
  geo_point2.altitude = 0.0;

  geodesy::ECEFPoint pt2(geo_point2);

  EXPECT_TRUE(geodesy::isValid(pt2));
  EXPECT_NEAR(pt2.x, pt.x, 0.01);
  EXPECT_NEAR(pt2.y, pt.y, 0.01);
  EXPECT_NEAR(pt2.z, pt.z, 0.01);
}

TEST(ECEFPoint, symmetry)
{
  geographic_msgs::msg::GeoPoint gp;
  gp.latitude = 43.0;
  gp.longitude = 75.0;
  gp.altitude = 0.0;

  geographic_msgs::msg::GeoPoint gp2;
  gp2.latitude = -gp.latitude;  // Symmetric latitude
  gp2.longitude = gp.longitude;
  gp2.altitude = gp.altitude;

  geographic_msgs::msg::GeoPoint gp3;
  gp3.latitude = gp.latitude;  // Symmetric latitude
  gp3.longitude = -gp.longitude;
  gp3.altitude = gp.altitude;

  geodesy::ECEFPoint pt(gp);
  geodesy::ECEFPoint pt2(gp2);
  geodesy::ECEFPoint pt3(gp3);

  EXPECT_TRUE(geodesy::isValid(pt));
  EXPECT_TRUE(geodesy::isValid(pt2));
  EXPECT_TRUE(geodesy::isValid(pt3));

  // Check symmetry
  EXPECT_NEAR(pt.x, pt2.x, 0.01);  // X should be the same
  EXPECT_NEAR(pt.y, pt2.y, 0.01);  // Y should be the same
  EXPECT_NEAR(pt.z, -pt2.z, 0.01);  // Z should be opposite

  EXPECT_NEAR(pt.x, pt3.x, 0.01);  // X should be the same
  EXPECT_NEAR(pt.y, -pt3.y, 0.01);  // Y should be opposite
  EXPECT_NEAR(pt.z, pt3.z, 0.01);  // Z should be the same
}

TEST(GeoPoint, centerOfEarth)
{
  // Center of the Earth
  geodesy::ECEFPoint pt(0.0, 0.0, 0.0);

  EXPECT_EQ(pt.x, 0.0);
  EXPECT_EQ(pt.y, 0.0);
  EXPECT_EQ(pt.z, 0.0);
  EXPECT_TRUE(geodesy::isValid(pt));

  geographic_msgs::msg::GeoPoint geo_point;
  convert(pt, geo_point);
  EXPECT_FALSE(geodesy::isValid(geo_point));
}

TEST(GeoPoint, fromECEF)
{
  geodesy::ECEFPoint pt(1541491.43, -4404522.62, 4333366.08);

  geographic_msgs::msg::GeoPoint geo_point;
  convert(pt, geo_point);

  EXPECT_TRUE(geodesy::isValid(geo_point));
  EXPECT_NEAR(geo_point.latitude, 43.072220, 0.000001);
  EXPECT_NEAR(geo_point.longitude, -70.711014, 0.000001);
  EXPECT_NEAR(geo_point.altitude, 0.0, 0.01);
}


TEST(GeoPoint, fromECEFAboveEllipsoid)
{
  geodesy::ECEFPoint pt(1541551.76, -4404694.99, 4333536.81);

  geographic_msgs::msg::GeoPoint geo_point;
  convert(pt, geo_point);

  EXPECT_TRUE(geodesy::isValid(geo_point));
  EXPECT_NEAR(geo_point.latitude, 43.072220, 0.000001);
  EXPECT_NEAR(geo_point.longitude, -70.711014, 0.000001);
  EXPECT_NEAR(geo_point.altitude, 250.0, 0.01);
}

TEST(GeoPoint, fromECEFBelowEllipsoid)
{
  geodesy::ECEFPoint pt(1541370.78, -4404177.87, 4333024.62);

  geographic_msgs::msg::GeoPoint geo_point;
  convert(pt, geo_point);

  EXPECT_TRUE(geodesy::isValid(geo_point));
  EXPECT_NEAR(geo_point.latitude, 43.072220, 0.000001);
  EXPECT_NEAR(geo_point.longitude, -70.711014, 0.000001);
  EXPECT_NEAR(geo_point.altitude, -500.0, 0.01);
}

//////////////
// ECEFPose //
//////////////

// Test null constructor
TEST(ECEFPose, nullConstructor)
{
  geodesy::ECEFPose pt;

  EXPECT_EQ(std::isnan(pt.position.x), true);
  EXPECT_EQ(std::isnan(pt.position.y), true);
  EXPECT_EQ(std::isnan(pt.position.z), true);
  EXPECT_EQ(pt.orientation.x, 0.0);
  EXPECT_EQ(pt.orientation.y, 0.0);
  EXPECT_EQ(pt.orientation.z, 0.0);
  EXPECT_EQ(pt.orientation.w, 1.0);

  EXPECT_FALSE(geodesy::isValid(pt));
}

TEST(ECEFPose, constructorFromECEFPointAndQuaternion)
{
  geodesy::ECEFPoint position(1541491.43, -4404522.62, 4333366.08);
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 1.0;

  geodesy::ECEFPose pose(position, orientation);

  EXPECT_EQ(pose.position.x, position.x);
  EXPECT_EQ(pose.position.y, position.y);
  EXPECT_EQ(pose.position.z, position.z);
  EXPECT_EQ(pose.orientation.x, orientation.x);
  EXPECT_EQ(pose.orientation.y, orientation.y);
  EXPECT_EQ(pose.orientation.z, orientation.z);
  EXPECT_EQ(pose.orientation.w, orientation.w);

  EXPECT_TRUE(geodesy::isValid(pose));
}

TEST(ECEFPose, constructorFromGeoPose)
{
  geographic_msgs::msg::GeoPose geo_pose;
  geo_pose.position.latitude = 43.072220;
  geo_pose.position.longitude = -70.711014;
  geo_pose.position.altitude = 0.0;
  geo_pose.orientation.x = 0.0;
  geo_pose.orientation.y = 0.0;
  geo_pose.orientation.z = 0.0;
  geo_pose.orientation.w = 1.0;

  geodesy::ECEFPose pose(geo_pose);

  EXPECT_TRUE(geodesy::isValid(pose));
  EXPECT_NEAR(pose.position.x, 1541491.43, 0.01);
  EXPECT_NEAR(pose.position.y, -4404522.62, 0.01);
  EXPECT_NEAR(pose.position.z, 4333366.08, 0.01);
  EXPECT_EQ(pose.orientation.x, 0.0);
  EXPECT_EQ(pose.orientation.y, 0.0);
  EXPECT_EQ(pose.orientation.z, 0.0);
  EXPECT_EQ(pose.orientation.w, 1.0);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
