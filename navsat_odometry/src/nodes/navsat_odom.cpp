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
#include <geodesy/utm.h>

#include "navsat_odom.h"

/** @file

    @brief Class for generating odometry from navigation satellite and
           inertial measurement data.

@par Subscribes

 - @b gps (sensor_msgs/NavSatFix): Satellite position fix.

 - @b imu (sensor_msgs/Imu): Inertial measurements.

@par Publishes

 - @b odom (nav_msgs/Odometry): Current estimate of robot position and
velocity in three dimensions, including roll, pitch, and yaw.  All
position data are in UTM coordinates.

 - @b tf: broadcast transform from @b /odom frame to @b /base_link.
   (The actual frame IDs come from the GPS and IMU messages.)

*/

using namespace navsat_odom;

/** Navigation satellite odometry constructor. */
NavSatOdom::NavSatOdom(ros::NodeHandle node, ros::NodeHandle priv_nh):
  node_(node),
  priv_nh_(priv_nh)
{
  // Connect to ROS topics, no delay: always the most recent data.
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  gps_sub_ =
    node_.subscribe("gps", 1, &NavSatOdom::processImu, this, noDelay);
  imu_sub_ =
    node_.subscribe("imu", 1, &NavSatOdom::processGps, this, noDelay);

  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1);
  // setup tf::TransformBroadcaster odom_broadcaster_;
}

/** Navigation satellite message callback. */
void NavSatOdom::processGps(const sensor_msgs::NavSatFix::ConstPtr &msgIn)
{
  gps_prev_ = gps_msg_;                 // save previous message
  gps_msg_ = *msgIn;

  if (haveNewData())
    publishOdom();
}

/** Inertial measurement message callback. */
void NavSatOdom::processImu(const sensor_msgs::Imu::ConstPtr &msgIn)
{
  imu_prev_ = imu_msg_;                 // save previous message
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

  // Use the most recent input message time stamp, get odometry frame
  // from GPS message, child frame from IMU message.
  pub_time_ = gps_msg_.header.stamp;
  if (imu_msg_.header.stamp > pub_time_)
    pub_time_ = imu_msg_.header.stamp;
  msg->header.stamp = pub_time_;
  msg->header.frame_id = gps_msg_.header.frame_id;
  msg->child_frame_id = imu_msg_.header.frame_id;;

  /// @todo Provide options to override the GPS and IMU frame IDs
  /// (using tf_prefix, if defined).

  // Convert the GPS message to a WGS 84 latitude/longitude pose, then
  // to a UTM pose.
  geographic_msgs::GeoPose ll_pose(geodesy::toMsg(gps_msg_,
                                                  imu_msg_.orientation));
  geodesy::UTMPose utm_pose(ll_pose);

  if (isValid(prev_pose_))              // not the first time through?
    {
      // Check the UTM grid zone designator, warn if it changes.
      if (!sameGridZone(prev_pose_, utm_pose))
        {
          /// @todo Add operator<< for UTMPoint and UTMPose
          ROS_WARN_STREAM_THROTTLE(30, "leaving previous UTM grid zone");

          /// @todo Now, what?
        }

      /// convert the current UTM pose to a geometry_msgs::Pose
      msg->pose.pose = toGeometry(utm_pose);

      // Copy the (3x3) position covariance to the upper left corner
      // of the (6x6) pose covariance.
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          {
            msg->pose.covariance[6*i+j] = gps_msg_.position_covariance[3*i+j];
          }

      // Unpack IMU data.  Copy the (3x3) orientation covariance to
      // the lower right corner of the (6x6) pose covariance.  Also
      // copy the (3x3) angular velocity covariance to the lower right
      // corner of the (6x6) twist covariance.
      msg->twist.twist.angular = imu_msg_.angular_velocity;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          {
            msg->pose.covariance[6*(i+3)+(j+3)] =
              imu_msg_.orientation_covariance[3*i+j];
            msg->twist.covariance[6*(i+3)+(j+3)] =
              imu_msg_.angular_velocity_covariance[3*i+j];
          }

      /// @todo Since twist.twist.linear is not available, compute
      /// position change since the previous pose divided by the time
      /// between the last two GPS messages.  Although position is in
      /// the parent frame, twist must be in the child frame.

      odom_pub_.publish(msg);

      // publish transform from /odom to /base_link
      tf::Pose odom_tf;
      tf::poseMsgToTF(msg->pose.pose, odom_tf);
      tf::StampedTransform tfMsg(odom_tf, pub_time_,
                                 msg->header.frame_id,
                                 msg->child_frame_id);
      odom_broadcaster_.sendTransform(tfMsg);
    }

  // track previous UTM pose
  prev_pose_ = utm_pose;
}
