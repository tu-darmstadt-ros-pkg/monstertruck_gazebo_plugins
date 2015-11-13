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
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */

#include <algorithm>
#include <assert.h>
#include <cmath>

#include <monstertruck_gazebo_plugins/all_wheel_steering_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
  #define RADIAN Radian
#else
  #define RADIAN GetAsRadian
#endif

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin)

enum
{
  FRONT_RIGHT = 0, FRONT_LEFT = 1, REAR_RIGHT = 2, REAR_LEFT = 3
};

// Constructor
AllWheelSteeringPlugin::AllWheelSteeringPlugin()
{
  rosnode_ = 0;
}

// Destructor
AllWheelSteeringPlugin::~AllWheelSteeringPlugin()
{
  sub_.shutdown();
  delete rosnode_;
}

// Load the controller
void AllWheelSteeringPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // get physics
  model = _model;
  world = _model->GetWorld();

  // default parameters
  topicName = "drive";
  odomTopicName = "odom";
  jointStateName = "joint_states";
  proportionalControllerGain = 8.0;
  derivativeControllerGain = 0.0;
  wheelTrack = 0.24;
  wheelBase = 0.30;
  wheelRadius = 0.14;
  jointMaxTorque = 10.0;
  wheelMaxTorque = 10.0;
  minRadius = 0.3;
  maxVelX = 0.4;

  // load parameters
  if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->Get<std::string>("robotNamespace");
  if (_sdf->HasElement("topicName")) topicName = _sdf->Get<std::string>("topicName");
  if (_sdf->HasElement("odomTopicName")) odomTopicName = _sdf->Get<std::string>("odomTopicName");
  if (_sdf->HasElement("jointStateName")) jointStateName = _sdf->Get<std::string>("jointStateName");
  if (_sdf->HasElement("frontLeftAxle")) wheels[FRONT_LEFT].axleName = _sdf->Get<std::string>("frontLeftAxle");
  if (_sdf->HasElement("frontRightAxle")) wheels[FRONT_RIGHT].axleName = _sdf->Get<std::string>("frontRightAxle");
  if (_sdf->HasElement("rearLeftAxle")) wheels[REAR_RIGHT].axleName = _sdf->Get<std::string>("rearLeftAxle");
  if (_sdf->HasElement("rearRightAxle")) wheels[REAR_LEFT].axleName = _sdf->Get<std::string>("rearRightAxle");
  if (_sdf->HasElement("frontLeftJoint")) wheels[FRONT_LEFT].jointName = _sdf->Get<std::string>("frontLeftJoint");
  if (_sdf->HasElement("frontRightJoint")) wheels[FRONT_RIGHT].jointName = _sdf->Get<std::string>("frontRightJoint");
  if (_sdf->HasElement("rearLeftJoint")) wheels[REAR_RIGHT].jointName = _sdf->Get<std::string>("rearLeftJoint");
  if (_sdf->HasElement("rearRightJoint")) wheels[REAR_LEFT].jointName = _sdf->Get<std::string>("rearRightJoint");
  if (_sdf->HasElement("proportionalControllerGain")) proportionalControllerGain = _sdf->Get<double>("proportionalControllerGain");
  if (_sdf->HasElement("derivativeControllerGain")) derivativeControllerGain = _sdf->Get<double>("derivativeControllerGain");
  if (_sdf->HasElement("wheelTrack")) wheelTrack = _sdf->Get<double>("wheelTrack");
  if (_sdf->HasElement("wheelBase")) wheelBase = _sdf->Get<double>("wheelBase");
  if (_sdf->HasElement("wheelRadius")) wheelRadius = _sdf->Get<double>("wheelRadius");
  if (_sdf->HasElement("jointMaxTorque")) jointMaxTorque = _sdf->Get<double>("jointMaxTorque");
  if (_sdf->HasElement("wheelMaxTorque")) wheelMaxTorque = _sdf->Get<double>("wheelMaxTorque");
  if (_sdf->HasElement("minRadius")) minRadius = _sdf->Get<double>("minRadius");
  if (_sdf->HasElement("maxVelX")) maxVelX = _sdf->Get<double>("maxVelX");

  double controlRate = 0.0;
  if (_sdf->HasElement("controlRate")) controlRate = _sdf->Get<double>("controlRate");
  controlPeriod = controlRate > 0.0 ? 1.0/controlRate : 0.0;

  for(int i = 0; i < 4; ++i) {
    wheels[i].axle  = _model->GetJoint(wheels[i].axleName);
    wheels[i].joint = _model->GetJoint(wheels[i].jointName);
  }

  if (!wheels[FRONT_LEFT].axle)
    gzthrow("The controller couldn't get front left axle");

  if (!wheels[FRONT_RIGHT].axle)
    gzthrow("The controller couldn't get front right axle");

  if (!wheels[REAR_LEFT].axle)
    gzthrow("The controller couldn't get rear left axle");

  if (!wheels[REAR_RIGHT].axle)
    gzthrow("The controller couldn't get rear right axle");

  if (!wheels[FRONT_LEFT].joint)
    gzthrow("The controller couldn't get front left hinge joint");

  if (!wheels[FRONT_RIGHT].joint)
    gzthrow("The controller couldn't get front right hinge joint");

  if (!wheels[REAR_LEFT].joint)
    gzthrow("The controller couldn't get rear left hinge joint");

  if (!wheels[REAR_RIGHT].joint)
    gzthrow("The controller couldn't get rear right hinge joint");

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  rosnode_ = new ros::NodeHandle(robotNamespace);

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  if (!topicName.empty()) {
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<monstertruck_msgs::MotionCommand>(topicName, 1,
                                                            boost::bind(&AllWheelSteeringPlugin::motionCommandCallback, this, _1),
                                                            ros::VoidPtr(), &queue_);
    sub_ = rosnode_->subscribe(so);
  }

  if (!odomTopicName.empty()) {
    odomPub_ = rosnode_->advertise<nav_msgs::Odometry>(odomTopicName, 10);
  }

  if (!jointStateName.empty()) {
    jointStatePub_ = rosnode_->advertise<sensor_msgs::JointState>(jointStateName, 10);
  }

  std::string tf_prefix = tf::getPrefixParam(*rosnode_);
  joint_state.header.frame_id = tf::resolve(tf_prefix, model->GetLink()->GetName());
  odom_.header.frame_id = tf::resolve(tf_prefix, "odom");
  odom_.child_frame_id = tf::resolve(tf_prefix, "base_footprint");

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AllWheelSteeringPlugin::Update, this));
}

// Initialize the controller
void AllWheelSteeringPlugin::Init()
{
  Reset();
}

// Reset
void AllWheelSteeringPlugin::Reset()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  enableMotors = false;

  wheels[FRONT_RIGHT].jointSpeed = 0;
  wheels[FRONT_LEFT].jointSpeed = 0;
  wheels[REAR_RIGHT].jointSpeed = 0;
  wheels[REAR_LEFT].jointSpeed = 0;

  wheels[FRONT_RIGHT].wheelSpeed = 0;
  wheels[FRONT_LEFT].wheelSpeed = 0;
  wheels[REAR_RIGHT].wheelSpeed = 0;
  wheels[REAR_LEFT].wheelSpeed = 0;

  prevUpdateTime = world->GetSimTime();
}

// Update the controller
void AllWheelSteeringPlugin::Update()
{
  // TODO: Step should be in a parameter of this function
  double a, b, r;
  double omega_fl, omega_fr, omega_rl, omega_rr, omega_phi;
  double phi_fl, phi_fr, phi_rl, phi_rr, R;
  double v;

  // handle callbacks
  queue_.callAvailable();

  a = wheelTrack / 2;
  b = wheelBase / 2;
  r = wheelRadius;

  common::Time stepTime;
  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = world->GetSimTime() - prevUpdateTime;

  if (controlPeriod == 0.0 || stepTime > controlPeriod) {
    GetPositionCmd();

    prevUpdateTime = world->GetSimTime();

    //TODO
    // odometry calculation
    omega_fl = wheels[FRONT_LEFT].axle->GetVelocity(0);
    omega_fr = wheels[FRONT_RIGHT].axle->GetVelocity(0);
    omega_rl = wheels[REAR_LEFT].axle->GetVelocity(0);
    omega_rr = wheels[REAR_RIGHT].axle->GetVelocity(0);

    phi_fl = wheels[FRONT_LEFT].joint->GetAngle(0).RADIAN();
    phi_fr = wheels[FRONT_RIGHT].joint->GetAngle(0).RADIAN();
    phi_rl = wheels[REAR_LEFT].joint->GetAngle(0).RADIAN();
    phi_rr = wheels[REAR_RIGHT].joint->GetAngle(0).RADIAN();

    if (phi_fl < -M_PI_2 || phi_fl > M_PI_2) {
      // gzthrow("phi_fl: %f out of bounds" << phi_fl);
    }
    if (phi_fr < -M_PI_2 || phi_fr > M_PI_2) {
      // gzthrow("phi_fr: %f out of bounds" << phi_fr);
    }
    if (phi_rl < -M_PI_2 || phi_rl > M_PI_2) {
      // gzthrow("phi_rl: %f out of bounds" << phi_rl);
    }
    if (phi_rr < -M_PI_2 || phi_rr > M_PI_2) {
      // gzthrow("phi_rr: %f out of bounds" << phi_rr);
    }

    R = (((b / tan(phi_fl)) + a) + ((b / tan(phi_fr)) - a) + ((b / tan(-phi_rl)) + a) + ((b / tan(-phi_rr)) - a)) / 4;

    //Calculate angular velocity for ICC which is same as angular velocity of vehicle
    omega_phi = (omega_fl * sin(phi_fl) * r / b); //+ omega_fr * sin(phi_fr) + omega_rl * sin(-phi_rl) + omega_rr * sin(-phi_rr)) * r / (4 * b); //omega Berechnung fuer linkes Vorderrad. Alle Raeder berechnen??

    v = r * (omega_fl + omega_fr)/2; //fuer geradesaus fahrt

    // Compute odometric pose
    odomPose[0] += v * stepTime.Double() * cos(odomPose[2]);
    odomPose[1] += v * stepTime.Double() * sin(odomPose[2]);
    odomPose[2] += omega_phi * stepTime.Double() * 0.9;  //TODO MAGIC NUMBER

    // Compute odometric instantaneous velocity
    odomVel[0] = v;
    odomVel[1] = 0.0;
    odomVel[2] = omega_phi;

    //  write_position_data();
    publish_odometry();
    publish_joint_states();
  }

  // calculate wheel speeds
  wheels[FRONT_LEFT].wheelSpeed  = cmd_.speed;
  wheels[FRONT_RIGHT].wheelSpeed = cmd_.speed;
  wheels[REAR_LEFT].wheelSpeed   = cmd_.speed;
  wheels[REAR_RIGHT].wheelSpeed  = cmd_.speed;

  // get current steering angles
  double current_phi_fl = wheels[FRONT_LEFT].joint->GetAngle(0).RADIAN();
  double current_phi_fr = wheels[FRONT_RIGHT].joint->GetAngle(0).RADIAN();
  double current_phi_rl = wheels[REAR_LEFT].joint->GetAngle(0).RADIAN();
  double current_phi_rr = wheels[REAR_RIGHT].joint->GetAngle(0).RADIAN();
  double steerAngleFront = (current_phi_fl + current_phi_fr) / 2.0;
  double steerAngleRear  = (current_phi_rl + current_phi_rr) / 2.0;

  if (steerAngleRear != -steerAngleFront) {
    // double omega = arctan((tan(steerAngleRear) - tan(steerAngleFront))/2.0); // drift angle
    double rx = ((tan(steerAngleRear) - tan(steerAngleFront)) / (tan(steerAngleRear) + tan(steerAngleFront))) * b;
    double ry = 2.0/(tan(steerAngleRear) + tan(steerAngleFront)) * b;
    double r  = sqrt(rx*rx+ry*ry);

    wheels[FRONT_LEFT].wheelSpeed  *= sqrt((rx - b)*(rx - b) + (ry - a)*(ry - a)) / r;
    wheels[FRONT_RIGHT].wheelSpeed *= sqrt((rx - b)*(rx - b) + (ry + a)*(ry + a)) / r;
    wheels[REAR_LEFT].wheelSpeed   *= sqrt((rx + b)*(rx + b) + (ry - a)*(ry - a)) / r;
    wheels[REAR_RIGHT].wheelSpeed  *= sqrt((rx + b)*(rx + b) + (ry + a)*(ry + a)) / r;
  }

  ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel speeds:\n"
    << "front left:  " << wheels[FRONT_LEFT].wheelSpeed << "\n"
    << "front right: " << wheels[FRONT_RIGHT].wheelSpeed << "\n"
    << "rear left:   " << wheels[REAR_LEFT].wheelSpeed << "\n"
    << "rear right:  " << wheels[REAR_RIGHT].wheelSpeed);

  ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel poses:\n"
                         << "front left:  " << wheels[FRONT_LEFT].joint->GetChild()->GetWorldPose() << "\n"
    << "front right: " << wheels[FRONT_RIGHT].joint->GetChild()->GetWorldPose() << "\n"
    << "rear left:   " << wheels[REAR_LEFT].joint->GetChild()->GetWorldPose() << "\n"
    << "rear right:  " << wheels[REAR_RIGHT].joint->GetChild()->GetWorldPose());

  if (enableMotors)
  {
    wheels[FRONT_LEFT].joint->SetVelocity(0, wheels[FRONT_LEFT].jointSpeed);
    wheels[FRONT_RIGHT].joint->SetVelocity(0, wheels[FRONT_RIGHT].jointSpeed);
    wheels[REAR_LEFT].joint->SetVelocity(0, wheels[REAR_LEFT].jointSpeed);
    wheels[REAR_RIGHT].joint->SetVelocity(0, wheels[REAR_RIGHT].jointSpeed);

    wheels[FRONT_LEFT].joint->SetMaxForce(0, jointMaxTorque);
    wheels[FRONT_RIGHT].joint->SetMaxForce(0, jointMaxTorque);
    wheels[REAR_LEFT].joint->SetMaxForce(0, jointMaxTorque);
    wheels[REAR_RIGHT].joint->SetMaxForce(0, jointMaxTorque);


    wheels[FRONT_LEFT].axle->SetVelocity(0, wheels[FRONT_LEFT].wheelSpeed / r);
    wheels[FRONT_RIGHT].axle->SetVelocity(0, wheels[FRONT_RIGHT].wheelSpeed / r);
    wheels[REAR_LEFT].axle->SetVelocity(0, wheels[REAR_LEFT].wheelSpeed / r);
    wheels[REAR_RIGHT].axle->SetVelocity(0, wheels[REAR_RIGHT].wheelSpeed / r);

    wheels[FRONT_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
    wheels[FRONT_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
    wheels[REAR_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
    wheels[REAR_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);

  } else {
    wheels[FRONT_LEFT].joint->SetMaxForce(0, 0.0);
    wheels[FRONT_RIGHT].joint->SetMaxForce(0, 0.0);
    wheels[REAR_LEFT].joint->SetMaxForce(0, 0.0);
    wheels[REAR_RIGHT].joint->SetMaxForce(0, 0.0);
    wheels[FRONT_LEFT].axle->SetMaxForce(0, 0.0);
    wheels[FRONT_RIGHT].axle->SetMaxForce(0, 0.0);
    wheels[REAR_LEFT].axle->SetMaxForce(0, 0.0);
    wheels[REAR_RIGHT].axle->SetMaxForce(0, 0.0);
  }
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void AllWheelSteeringPlugin::GetPositionCmd()
{
  boost::mutex::scoped_lock lock(mutex);

  double mR, a, b, kappa, t1_left, t1_right, t2, factor_left, factor_right, v_left, v_right;
  double set_phi_left, set_phi_right, current_phi_fl, current_phi_fr, current_phi_rl, current_phi_rr;
  double vel_phi_fl, vel_phi_fr, vel_phi_rl, vel_phi_rr;

  mR = minRadius;
  a = wheelTrack / 2;
  b = wheelBase / 2;

  //  if (x_ == 0.0 && rot_ != 0.0 ) {
  //    x_ = fabs(rot_ * mR);
  ////    ROS_INFO("robot was given a rotation command without an linear speed. x_ set to %f", x_);
  ////    ROS_INFO("kappa calulates to %f", (rot_ / x_));
  //  }

  //  kappa = (x_ != 0.0) ? rot_ / x_ : 0.0;  // inverse radius

  //  if (kappa >  1.0/ mR ) kappa =  1.0/mR;
  //  if (kappa < -1.0/ mR ) kappa = -1.0/mR;

  //  t1_left  = 1.0 - a * kappa;
  //  t1_right = 1.0 + a * kappa;
  //  t2  = b * kappa;
  //  factor_left  = sqrt((t1_left*t1_left)  + (t2*t2));
  //  factor_right = sqrt((t1_right*t1_right) + (t2*t2));

  if (cmd_.speed > maxVelX) {
    cmd_.speed = maxVelX;
  } else if (cmd_.speed < -maxVelX) {
    cmd_.speed = -maxVelX;
  }

  //  v_left  = cmd_.linear.x * factor_left;
  //  v_right = cmd_.linear.x * factor_right;

  current_phi_fl = wheels[FRONT_LEFT].joint->GetAngle(0).RADIAN();
  current_phi_fr = wheels[FRONT_RIGHT].joint->GetAngle(0).RADIAN();
  current_phi_rl = wheels[REAR_LEFT].joint->GetAngle(0).RADIAN();
  current_phi_rr = wheels[REAR_RIGHT].joint->GetAngle(0).RADIAN();

  //  ROS_INFO("i: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", current_phi_fl, current_phi_fr, current_phi_rl, current_phi_rr);
  //  ROS_INFO("s: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", set_phi_left, set_phi_right, set_phi_left, set_phi_right);

  vel_phi_fl = wheels[FRONT_LEFT].joint->GetVelocity(0);
  vel_phi_fr = wheels[FRONT_RIGHT].joint->GetVelocity(0);
  vel_phi_rl = wheels[REAR_LEFT].joint->GetVelocity(0);
  vel_phi_rr = wheels[REAR_RIGHT].joint->GetVelocity(0);

  wheels[FRONT_LEFT].jointSpeed  = ( ((cmd_.steerAngleFront - current_phi_fl) * proportionalControllerGain) - vel_phi_fl * derivativeControllerGain);
  wheels[FRONT_RIGHT].jointSpeed = ( ((cmd_.steerAngleFront - current_phi_fr) * proportionalControllerGain) - vel_phi_fr * derivativeControllerGain);
  wheels[REAR_LEFT].jointSpeed   = ( ((cmd_.steerAngleRear  - current_phi_rl) * proportionalControllerGain) - vel_phi_rl * derivativeControllerGain);
  wheels[REAR_RIGHT].jointSpeed  = ( ((cmd_.steerAngleRear  - current_phi_rr) * proportionalControllerGain) - vel_phi_rr * derivativeControllerGain);

  //  ROS_INFO("v: fl: [%f] fr: [%f] rl: [%f] rr: [%f]\n", wheels[FRONT_LEFT].jointSpeed, wheels[FRONT_RIGHT].jointSpeed, wheels[REAR_LEFT].jointSpeed, wheels[REAR_RIGHT].jointSpeed);

  //ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "X: [" << x_ << "] ROT: [" << rot_ << "]");

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true;

  ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", enableMotors);
}

// NEW: Store the velocities from the ROS message
void AllWheelSteeringPlugin::motionCommandCallback(const monstertruck_msgs::MotionCommand::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock lock(mutex);
  cmd_ = *cmd_msg;
}

// NEW: Update this to publish odometry topic
void AllWheelSteeringPlugin::publish_odometry()
{
  if (!odomPub_) return;

  // publish odom topic
  odom_.pose.pose.position.x = odomPose[0];
  odom_.pose.pose.position.y = odomPose[1];

  tf::Quaternion qt = tf::createQuaternionFromRPY(0.0, 0.0, fmod(odomPose[2] + M_PI, 2*M_PI) - M_PI);
  tf::quaternionTFToMsg(qt, odom_.pose.pose.orientation);

  odom_.twist.twist.linear.x = odomVel[0];
  odom_.twist.twist.linear.y = odomVel[1];
  odom_.twist.twist.angular.z = odomVel[2];

  odom_.header.stamp.sec = world->GetSimTime().sec;
  odom_.header.stamp.nsec = world->GetSimTime().nsec;

  odomPub_.publish(odom_);
}

void AllWheelSteeringPlugin::publish_joint_states()
{
  if (!jointStatePub_) return;

  joint_state.header.stamp.sec = world->GetSimTime().sec;
  joint_state.header.stamp.nsec = world->GetSimTime().nsec;
  joint_state.name.resize(8);
  joint_state.position.resize(8);
  joint_state.velocity.resize(8);
  joint_state.effort.resize(8);

  for (unsigned int i = 0; i < 4; i++) {
    joint_state.name[i] = wheels[i].joint->GetName();
    joint_state.position[i] = wheels[i].joint->GetAngle(0).RADIAN();
    joint_state.velocity[i] = wheels[i].joint->GetVelocity(0);
    joint_state.effort[i] = wheels[i].joint->GetForce(0u);
  }

  for (unsigned int i = 0; i < 4; i++) {
    joint_state.name[4+i] = wheels[i].axle->GetName();
    joint_state.position[4+i] = wheels[i].axle->GetAngle(0).RADIAN();
    joint_state.velocity[4+i] = wheels[i].axle->GetVelocity(0);
    joint_state.effort[4+i] = wheels[i].axle->GetForce(0u);
  }

  jointStatePub_.publish(joint_state);
}

} // namespace gazebo

