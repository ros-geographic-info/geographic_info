/* -*- mode: C++ -*- */
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

#ifndef _NAVSAT_ODOM_H_
#define _NAVSAT_ODOM_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geodesy/utm.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

/** @file

    @brief Class interface for generating odometry from navigation
           satellite and inertial measurement data.

*/

namespace navsat_odom
{

class NavSatOdom
{
public:

  // public methods
  NavSatOdom(ros::NodeHandle node, ros::NodeHandle priv_nh);
  ~NavSatOdom() {};

private:

  /** @return true if there are new fix and IMU data since the last
      publication. */
  bool haveNewData(void)
  {
    return ((pub_time_ < fix_msg_.header.stamp)
            && (pub_time_ < imu_msg_.header.stamp));
  }

  void processFix(const sensor_msgs::NavSatFix::ConstPtr &msgIn);
  void processImu(const sensor_msgs::Imu::ConstPtr &msgIn);
  void publishOdom(void);

  ros::NodeHandle node_;                // node handle
  ros::NodeHandle priv_nh_;             // private node handle

  ros::Subscriber fix_sub_;             // NavSatFix message subscriber
  ros::Subscriber imu_sub_;             // Imu message subscriber

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  sensor_msgs::NavSatFix fix_msg_;      ///< latest fix message
  sensor_msgs::NavSatFix fix_prev_;     ///< previous fix message
  geodesy::UTMPoint fix_msg_pt_;        ///< latest point [UTM]
  geodesy::UTMPoint fix_prev_pt_;       ///< previous point [UTM]

  sensor_msgs::Imu imu_msg_;            ///< latest IMU message
  sensor_msgs::Imu imu_prev_;           ///< previous IMU message

  ros::Time pub_time_;                  ///< latest publication time
  geodesy::UTMPose prev_pose_;          ///< previous pose [UTM]


}; // end class NavSatOdom

}; // end namespace navsat_odom

#endif // _NAVSAT_ODOM_H_
