/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ALL_WHEEL_STEERING_PLUGIN_H
#define ALL_WHEEL_STEERING_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <monstertruck_msgs/MotionCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class AllWheelSteeringPlugin : public ModelPlugin
{

public:
  AllWheelSteeringPlugin();
  virtual ~AllWheelSteeringPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Reset();
  virtual void Update();

private:
  void write_position_data();
  void publish_odometry();
  void publish_joint_states();
  void GetPositionCmd();

private:
  // physics
  physics::ModelPtr model;
  physics::WorldPtr world;

  // parameters
  std::string robotNamespace;
  std::string topicName;
  std::string odomTopicName;
  std::string jointStateName;

  common::Time controlPeriod;
  float proportionalControllerGain;
  float derivativeControllerGain;

  float wheelTrack;
  float wheelBase;
  float wheelRadius;
  float jointMaxTorque;
  float wheelMaxTorque;
  float minRadius;
  float maxVelX;

  struct Wheel {
    std::string axleName;
    std::string jointName;
    float jointSpeed;
    float wheelSpeed;
    physics::JointPtr axle;
    physics::JointPtr joint;
  } wheels[4];

  // current values
  bool enableMotors;
  float odomPose[3];
  float odomVel[3];
  nav_msgs::Odometry odom_;
  sensor_msgs::JointState joint_state;

  // Simulation time of the last update
  common::Time prevUpdateTime;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher odomPub_;
  ros::Publisher jointStatePub_;
  ros::Subscriber sub_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
//  boost::thread* callback_queue_thread_;
//  void QueueThread();

  // DiffDrive stuff
  boost::mutex mutex;
  void motionCommandCallback(const monstertruck_msgs::MotionCommand::ConstPtr& cmd_msg);
  monstertruck_msgs::MotionCommand cmd_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif
