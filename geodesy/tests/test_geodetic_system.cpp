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
#include <limits>
#include "geodesy/geodetic_system.h"

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test null point constructor
TEST(GeoPoint, nullConstructor)
{
  geographic_msgs::GeoPoint pt;

  EXPECT_FALSE(geodesy::is2D(pt));
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test point with no altitude
TEST(GeoPoint, noAltitude)
{
  geographic_msgs::GeoPoint pt;

  pt.altitude = std::numeric_limits<double>::signaling_NaN();
  EXPECT_TRUE(geodesy::is2D(pt));
}

// Test point with valid altitude
TEST(GeoPoint, hasAltitude)
{
  geographic_msgs::GeoPoint pt;

  pt.altitude = 200.0;
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = -100.0;
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = 20000.0;
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = 0.0;
  EXPECT_FALSE(geodesy::is2D(pt));
}

// Test valid latitudes and longitudes
TEST(GeoPoint, validLatLong)
{
  geographic_msgs::GeoPoint pt;

  pt.latitude = 90.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.latitude = -90.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.latitude = 30.0;
  pt.longitude = -97.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.longitude = 180.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.longitude = -180.0;
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test valid latitudes and longitudes
TEST(GeoPoint, invalidLatLong)
{
  geographic_msgs::GeoPoint pt;

  pt.latitude = 90.001;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.latitude = -90.3;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.latitude = 30.0;
  pt.longitude = -1000.0;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.longitude = 180.0001;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.longitude = -180.999;
  EXPECT_FALSE(geodesy::isValid(pt));
}

// Test null pose constructor
TEST(GeoPose, nullConstructor)
{
  geographic_msgs::GeoPose pose;

  EXPECT_FALSE(geodesy::is2D(pose));
  EXPECT_TRUE(geodesy::isValid(pose));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "geotypes_unit_test");
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
