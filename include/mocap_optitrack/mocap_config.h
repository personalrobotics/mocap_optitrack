/*
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: mocap_config.h
 *  Desc: Classes representing ROS configuration for mocap_optitrack node. Data
 *  will be published to differed topics based on the configuration provided.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2012, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#ifndef __MOCAP_CONFIG_H__
#define __MOCAP_CONFIG_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "mocap_datapackets.h"


class PublishedMarker
{
private:
  ros::NodeHandle n;

  std::string topic;
  std::string frame_id;

  ros::Publisher pub;

public:
  Marker currentMarker;
  int disconnectedFrames;
  PublishedMarker(XmlRpc::XmlRpcValue &);
  void publish();
  void update(Marker &);
};


typedef std::map<int, PublishedMarker> MarkerMap;
typedef std::pair<int, PublishedMarker> MarkerItem;
typedef std::vector<PublishedMarker> MarkerArray;

class PublishedPointArray
{
private:
  ros::NodeHandle n;
  std::string topic;
  std::string frame_id;
  ros::Publisher pub;
public:
  MarkerArray published_unlabeled_markers;
  MarkerArray published_model_markers; // No need to track manually
  void publish();
  PublishedPointArray(ros::NodeHandle &);
};

class PublishedRigidBody
{
  private:
  ros::NodeHandle n;

  std::string pose_topic;
  std::string pose2d_topic;
  std::string parent_frame_id;
  std::string child_frame_id;

  bool publish_pose;
  bool publish_tf;
  bool publish_pose2d;
  bool use_new_coordinates;
  bool publish_markers;

  tf::TransformBroadcaster tf_pub;
  ros::Publisher pose_pub;
  ros::Publisher pose2d_pub;

  int first_published_marker;
  int end_published_marker;

  bool validateParam(XmlRpc::XmlRpcValue &, const std::string &);

  public:
  PublishedRigidBody(XmlRpc::XmlRpcValue &, MarkerArray &);
  void updateMarker(Marker*, int, MarkerArray &);
  void publish(RigidBody &, MarkerArray &);
};

typedef std::map<int, PublishedRigidBody> RigidBodyMap;
typedef std::pair<int, PublishedRigidBody> RigidBodyItem;

#endif  // __MOCAP_CONFIG_H__
