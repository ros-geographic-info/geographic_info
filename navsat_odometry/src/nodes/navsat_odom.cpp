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
   (The actual frame IDs come from the fix and IMU messages.)

*/

using namespace navsat_odom;

/** Navigation satellite odometry constructor. */
NavSatOdom::NavSatOdom(ros::NodeHandle node, ros::NodeHandle priv_nh):
  node_(node),
  priv_nh_(priv_nh)
{
  // Connect to ROS topics, no delay: always the most recent data.
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  fix_sub_ =
    node_.subscribe("gps", 1, &NavSatOdom::processFix, this, noDelay);
  imu_sub_ =
    node_.subscribe("imu", 1, &NavSatOdom::processImu, this, noDelay);

  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1);
  // setup tf::TransformBroadcaster odom_broadcaster_;
}

/** Navigation satellite message callback. */
void NavSatOdom::processFix(const sensor_msgs::NavSatFix::ConstPtr &msgIn)
{
  // save previous message and its UTM point
  fix_prev_ = fix_msg_;
  fix_prev_pt_ = fix_msg_pt_;

  // save latest message, convert latitude and longitude to a UTM point
  fix_msg_ = *msgIn;
  geographic_msgs::GeoPoint latlon(geodesy::toMsg(fix_msg_));
  convert(latlon, fix_msg_pt_);

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
 *  @pre Both fix_msg_ and imu_msg_ contain recent data to publish.
 */
void NavSatOdom::publishOdom(void)
{
  // allocate shared pointer to enable zero-copy publication
  boost::shared_ptr<nav_msgs::Odometry> msg(new nav_msgs::Odometry);

  // Use the most recent input message time stamp, get odometry frame
  // from fix message, child frame from IMU message.
  pub_time_ = fix_msg_.header.stamp;
  if (imu_msg_.header.stamp > pub_time_)
    pub_time_ = imu_msg_.header.stamp;
  msg->header.stamp = pub_time_;
  msg->header.frame_id = fix_msg_.header.frame_id;
  msg->child_frame_id = imu_msg_.header.frame_id;;

  /// @todo Provide options to override the fix and IMU frame IDs
  /// (using tf_prefix, if defined).

  // Create a UTM pose from the latest fix and the latest IMU
  // orientation.
  geodesy::UTMPose utm_pose(fix_msg_pt_, imu_msg_.orientation);

  if (isValid(prev_pose_))              // not the first time through?
    {
      // Check the UTM grid zone designator, warn if it changes.
      if (!sameGridZone(prev_pose_, utm_pose))
        {
          ROS_WARN_STREAM_THROTTLE(30, "leaving UTM grid zone "
                                   << prev_pose_.position
                                   << ", entering "
                                   << utm_pose.position);
          /// @todo The robot just entered a new UTM grid zone. Now, what?
        }

      /// convert the current UTM pose to a geometry_msgs::Pose
      msg->pose.pose = toGeometry(utm_pose);

      // Copy the (3x3) position covariance to the upper left corner
      // of the (6x6) pose covariance.
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          {
            msg->pose.covariance[6*i+j] = fix_msg_.position_covariance[3*i+j];
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

      // publish transform from /odom to /base_link
      tf::Pose odom_tf;
      tf::poseMsgToTF(msg->pose.pose, odom_tf);
      tf::StampedTransform tfMsg(odom_tf, pub_time_,
                                 msg->header.frame_id,
                                 msg->child_frame_id);
      odom_broadcaster_.sendTransform(tfMsg);

      // Since twist.twist.linear is not directly available, compute
      // position change since the previous point divided by the time
      // between the last two fix messages.  Although position is in
      // the parent frame, twist must be reported in the child frame.
      btVector3 pt0(toBullet(fix_prev_pt_));
      btVector3 pt1(toBullet(fix_msg_pt_));
      ros::Duration dt(fix_msg_.header.stamp - fix_prev_.header.stamp);
      btVector3 vel((pt1 - pt0) / dt.toSec());
      /// @todo transform velocity into child frame
      tf::vector3TFToMsg(vel, msg->twist.twist.linear);

      odom_pub_.publish(msg);
    }

  // track previous UTM pose
  prev_pose_ = utm_pose;
}
