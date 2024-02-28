// Copyright (c) 2024
// AeroVironment, Inc. All rights reserved.

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// THIS SOFTWARE IS PROVIDED BY [Name of Organization] “AS IS” AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AeroVironment, Inc.
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/validation.hpp>


TEST(ElevationServerCore, ValidPoints)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 1.0;
  point.longitude = 10.0;
  EXPECT_TRUE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, TooFarNorth)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 90.01;
  point.longitude = 0.0;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, TooFarSouth)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = -90.01;
  point.longitude = 0.0;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, TooFarWest)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 0.0;
  point.longitude = -180.1;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, TooFarEast)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 0.0;
  point.longitude = 180.1;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, NorthPole)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 90.0;
  point.longitude = 42.0;
  EXPECT_TRUE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, SouthPole)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = -90.0;
  point.longitude = 42.0;
  EXPECT_TRUE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, NorthPoleInvalidLng)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = 90.0;
  point.longitude = 999.0;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}

TEST(ElevationServerCore, SouthPoleInvalidLng)
{
  geographic_msgs::msg::GeoPoint point;
  point.latitude = -90.0;
  point.longitude = -999.0;
  EXPECT_FALSE(geographic_msgs::horizontalPositionValid(point));
}
