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

#include "navsat_odom.h"

/** @file

    @brief ROS class for generating odometry from navigation satellite
           data.

@par Subscribes

 - @b gps (sensor_msgs/NavSatFix): Satellite position fix.

 - @b imu (sensor_msgs/Imu): Inertial measurements.

@par Publishes

 - @b odom (nav_msgs/Odometry): Current estimate of robot position and
velocity in three dimensions, including roll, pitch, and yaw.  All
data are in the @b /odom frame of reference, location in UTM coordinates.

 - @b tf: broadcast transform from @b /base_link frame to @b /odom frame.

*/

using namespace navsat_odom;

/** Navigation satellite odometry constructor. */
NavSatOdom::NavSatOdom(ros::NodeHandle node, ros::NodeHandle priv_nh):
  node_(node),
  priv_nh_(priv_nh)
{
  // connect to ROS topics
  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  gps_sub_ =
    node_.subscribe("gps", 1, &NavSatOdom::processImu, this, noDelay);
  imu_sub_ =
    node_.subscribe("imu", 1, &NavSatOdom::processGps, this, noDelay);

  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1);
  // setup tf::TransformBroadcaster odom_broadcaster_;
}

/** @return true if there are new GPS and IMU data to publish. */
bool NavSatOdom::haveNewData(void)
{
  /// @todo check that both messages have arrived recently
  return true;
}

/** Navigation satellite message callback. */
void NavSatOdom::processGps(const sensor_msgs::NavSatFix::ConstPtr &msgIn)
{
  gps_msg_ = *msgIn;

  if (haveNewData())
    publishOdom();
}

/** Inertial measurement message callback. */
void NavSatOdom::processImu(const sensor_msgs::Imu::ConstPtr &msgIn)
{
  imu_msg_ = *msgIn;

  if (haveNewData())
    publishOdom();
}

/** Publish odometry and transforms.
 *
 *  @pre Both gps_msg_ and imu_msg_ contain recent data to publish.
 */
void NavSatOdom::publishOdom(void)
{
  // allocate shared pointer to enable zero-copy publication
  boost::shared_ptr<nav_msgs::Odometry> msg(new nav_msgs::Odometry);

  // use the most recent input message time stamp
  pub_time_ = gps_msg_.header.stamp;
  if (imu_msg_.header.stamp > pub_time_)
    pub_time_ = imu_msg_.header.stamp;
  msg->header.stamp = pub_time_;

  /// @todo use tf_prefix, if defined
  msg->header.frame_id = "/odom";
  msg->child_frame_id = "/base_link";

  odom_pub_.publish(msg);
}
