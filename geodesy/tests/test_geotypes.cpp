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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geographic_msgs/GeoPoint.h>
#include "geodesy/geotypes.h"

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test null constructor
TEST(GeoTypes, nullConstructor)
{
  geodesy::GridPoint gpt;

  EXPECT_EQ(gpt.easting, 0.0);
  EXPECT_EQ(gpt.northing, 0.0);
  EXPECT_TRUE(gpt.isFlat());
  EXPECT_EQ(gpt.zone, 0);
  EXPECT_EQ(gpt.band, ' ');
}

// Test flat constructor
TEST(GeoTypes, flatConstructor)
{
  double e = 1000.0;
  double n = 2400.0;
  uint8_t z = 14;
  char b = 'R';
  geodesy::GridPoint gpt(e, n, z, b);

  EXPECT_EQ(gpt.easting, e);
  EXPECT_EQ(gpt.northing, n);
  EXPECT_TRUE(gpt.isFlat());
  EXPECT_EQ(gpt.zone, z);
  EXPECT_EQ(gpt.band, b);
}

// Test 3D constructor
TEST(GeoTypes, altConstructor)
{
  double e = 1000.0;
  double n = 2400.0;
  double a = 200.0;
  uint8_t z = 14;
  char b = 'R';
  geodesy::GridPoint gpt(e, n, a, z, b);

  EXPECT_EQ(gpt.easting, e);
  EXPECT_EQ(gpt.northing, n);
  EXPECT_FALSE(gpt.isFlat());
  EXPECT_EQ(gpt.zone, z);
  EXPECT_EQ(gpt.band, b);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "geotypes_unit_test");
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
