/* $Id$ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011 Jack O'Quin
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
#include "geodesy/utm.h"

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test null constructor
TEST(UTMPoint, nullConstructor)
{
  geodesy::UTMPoint pt;

  EXPECT_EQ(pt.easting, 0.0);
  EXPECT_EQ(pt.northing, 0.0);
  EXPECT_TRUE(geodesy::isFlat(pt));
  EXPECT_EQ(pt.zone, 0);
  EXPECT_EQ(pt.band, ' ');
  EXPECT_FALSE(geodesy::isValid(pt));
}

// Test flat constructor
TEST(UTMPoint, flatConstructor)
{
  double e = 1000.0;
  double n = 2400.0;
  uint8_t z = 14;
  char b = 'R';
  geodesy::UTMPoint pt(e, n, z, b);

  EXPECT_EQ(pt.easting, e);
  EXPECT_EQ(pt.northing, n);
  EXPECT_TRUE(geodesy::isFlat(pt));
  EXPECT_EQ(pt.zone, z);
  EXPECT_EQ(pt.band, b);
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test 3D constructor
TEST(UTMPoint, hasAltitude)
{
  double e = 1000.0;
  double n = 2400.0;
  double a = 200.0;
  uint8_t z = 14;
  char b = 'R';
  geodesy::UTMPoint pt(e, n, a, z, b);

  EXPECT_EQ(pt.easting, e);
  EXPECT_EQ(pt.northing, n);
  EXPECT_FALSE(geodesy::isFlat(pt));
  EXPECT_EQ(pt.zone, z);
  EXPECT_EQ(pt.band, b);
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test copy constructor
TEST(UTMPoint, copyConstructor)
{
  double e = 1000.0;
  double n = 2400.0;
  double a = 200.0;
  uint8_t z = 14;
  char b = 'R';
  geodesy::UTMPoint pt1(e, n, a, z, b);
  geodesy::UTMPoint pt2(pt1);

  EXPECT_EQ(pt2.easting, e);
  EXPECT_EQ(pt2.northing, n);
  EXPECT_FALSE(geodesy::isFlat(pt2));
  EXPECT_EQ(pt2.zone, z);
  EXPECT_EQ(pt2.band, b);
  EXPECT_TRUE(geodesy::isValid(pt2));
}

// Test zone numbers
TEST(UTMPoint, testZones)
{
  geodesy::UTMPoint pt;
  pt.band = 'X';                        // supply a valid band letter

  pt.zone = 0;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.zone = 61;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.zone = 255;
  EXPECT_FALSE(geodesy::isValid(pt));

  // these should all work
  for (uint8_t b = 1; b <= 60; ++b)
    {
      pt.zone = b;
      EXPECT_TRUE(geodesy::isValid(pt));
    }
}

// Test band letters
TEST(UTMPoint, testBands)
{
  geodesy::UTMPoint pt;
  pt.zone = 14;                         // supply a valid zone number
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = '9';
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = ';';
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = 'I';
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = 'O';
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = 'Y';
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.band = 'r';
  EXPECT_FALSE(geodesy::isValid(pt));

  // this should work
  pt.band = 'X';
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test null pose constructor
TEST(UTMPose, nullConstructor)
{
  geodesy::UTMPose pose;

  EXPECT_TRUE(geodesy::isFlat(pose));
  EXPECT_FALSE(geodesy::isValid(pose));
}

// Test constructor from UTMPoint and a Quaternion
TEST(UTMConverson, fromLatLongToUtm)
{
  // University of Texas, Austin, Pickle Research Campus
  double lat = 30.385315;
  double lon = -97.728524;
  double alt = 209.0;

  geographic_msgs::GeoPoint ll;
  ll.latitude = lat;
  ll.longitude = lon;
  ll.altitude = alt;

  // convert point to UTM
  geodesy::UTMPoint pt(ll);

  double e = 622159.34;
  double n = 3362168.30;
  uint8_t z = 14;
  char b = 'R';
  double abs_err = 0.01;

  EXPECT_FALSE(geodesy::isFlat(pt));
  EXPECT_TRUE(geodesy::isValid(pt));

  EXPECT_NEAR(pt.easting, e, abs_err);
  EXPECT_NEAR(pt.northing, n, abs_err);
  EXPECT_NEAR(pt.altitude, alt, abs_err);
  EXPECT_EQ(pt.zone, z);
  EXPECT_EQ(pt.band, b);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "geotypes_unit_test");
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
