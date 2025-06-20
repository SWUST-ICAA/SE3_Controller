/****************************************************************************
 *
 *   Copyright (c) 2025 Nanwan. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

// ROS related headers
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

// Standard libraries
#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

// ROS message types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

// Third-party libraries
#include <Eigen/Dense>

// Custom messages and configurations
#include <mavros_controllers/FlatTarget.h>
#include <dynamic_reconfigure/server.h>
#include <mavros_controllers/GeometricControllerConfig.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

// Project internal headers
#include "mavros_controllers/common.h"
#include "mavros_controllers/control.h"

// Controller type definitions
#define ERROR_QUATERNION 1  // Quaternion error control
#define ERROR_GEOMETRIC 2   // Geometric error control
#define JERK_TRACKING 3     // Jerk tracking control

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

/**
 * @class MavrosControllers
 * @brief Implementation of MAVROS-based multirotor controller
 * 
 * This class provides functionality for position control, attitude control
 * and trajectory tracking, supporting multiple control modes
 */
class MavrosControllers {
 private:
  // ROS related members
  ros::NodeHandle nh_;                    // Public node handle
  ros::NodeHandle nh_private_;            // Private node handle
  
  // Subscribers
  ros::Subscriber referenceSub_;          // Reference trajectory subscriber
  ros::Subscriber flatreferenceSub_;      // Flat output reference subscriber
  ros::Subscriber multiDOFJointSub_;      // Multi-DOF joint trajectory subscriber
  ros::Subscriber mavstateSub_;           // MAV state subscriber
  ros::Subscriber mavposeSub_;            // MAV pose subscriber
  ros::Subscriber mavtwistSub_;           // MAV velocity subscriber
  ros::Subscriber yawreferenceSub_;       // Yaw reference subscriber
  
  // Publishers
  ros::Publisher angularVelPub_;          // Angular velocity command publisher
  ros::Publisher target_pose_pub_;        // Target pose publisher
  ros::Publisher referencePosePub_;       // Reference pose publisher
  ros::Publisher posehistoryPub_;         // Pose history publisher
  ros::Publisher systemstatusPub_;        // System status publisher
  
  // Service clients and servers
  ros::ServiceClient arming_client_;      // Arming service client
  ros::ServiceClient set_mode_client_;    // Set mode service client
  ros::ServiceServer ctrltriggerServ_;    // Control trigger service
  ros::ServiceServer land_service_;       // Landing service
  
  // Timers
  ros::Timer cmdloop_timer_;              // Command loop timer
  ros::Timer statusloop_timer_;           // Status loop timer
  
  // Time stamps
  ros::Time last_request_;                // Last request time
  ros::Time reference_request_now_;       // Current reference request time
  ros::Time reference_request_last_;      // Last reference request time
  ros::Time start_time_;                  // Integral control start time

  // Control parameters
  bool feedthrough_enable_{false};        // Feedthrough enable flag
  int ctrl_mode_;                         // Control mode
  bool auto_takeoff;                      // Auto takeoff flag
  bool velocity_yaw_;                     // Velocity yaw flag
  bool enable_integral_;                  // Integral term enable flag
  double reference_request_dt_;           // Reference request time interval
  double norm_thrust_const_;              // Normalized thrust constant
  double norm_thrust_offset_;             // Normalized thrust offset
  double max_fb_acc_;                     // Maximum feedback acceleration
  double max_int_;                        // Integral limit value
  int posehistory_window_;               // Pose history window size

  // State variables
  mavros_msgs::State current_state_;      // Current MAV state
  mavros_msgs::CommandBool arm_cmd_;      // Arm command
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;  // Pose history record
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;     // Companion state

  // Target state vectors
  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial target position
  Eigen::Vector3d targetPos_;             // Target position
  Eigen::Vector3d targetVel_;             // Target velocity
  Eigen::Vector3d targetAcc_;             // Target acceleration
  Eigen::Vector3d targetPos_prev_;        // Previous target position
  Eigen::Vector3d targetVel_prev_;        // Previous target velocity

  // MAV state vectors
  Eigen::Vector3d mavPos_;                // MAV position
  Eigen::Vector3d mavVel_;                // MAV velocity
  Eigen::Vector3d mavRate_;               // MAV angular rate
  double mavYaw_;                         // MAV yaw angle
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};  // Gravity vector
  Eigen::Vector4d mavAtt_;                // MAV attitude quaternion
  Eigen::Vector4d q_des;                  // Desired attitude quaternion

  // Control outputs and gains
  Eigen::Vector4d cmdBodyRate_;           // Body frame angular rate command {wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_;                  // Position gain vector
  Eigen::Vector3d Kvel_;                  // Velocity gain vector
  Eigen::Vector3d D_;                     // Damping vector
  Eigen::Vector3d Kint_;                  // Integral gain vector
  
  // Component gains
  double Kpos_x_, Kpos_y_, Kpos_z_;      // Position gain components
  double Kvel_x_, Kvel_y_, Kvel_z_;      // Velocity gain components
  double Kint_x_, Kint_y_, Kint_z_;      // Integral gain components
  
  // Control states
  Eigen::Vector3d pos_int_;              // Position integral error

  /**
   * @brief Publish angular velocity commands and target attitude
   * @param cmd Angular velocity command vector
   * @param target_attitude Target attitude quaternion
   */
  void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);

  /**
   * @brief Publish reference pose
   * @param target_position Target position
   * @param target_attitude Target attitude
   */
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);

  /**
   * @brief Publish pose history
   */
  void pubPoseHistory();

  /**
   * @brief Publish system status
   */
  void pubSystemStatus();

  /**
   * @brief Add current pose to history
   */
  void appendPoseHistory();

  /**
   * @brief Target trajectory callback
   * @param msg Velocity timestamped message
   */
  void targetCallback(const geometry_msgs::TwistStamped &msg);

  /**
   * @brief Flat output target callback
   * @param msg Flat target message
   */
  void flattargetCallback(const mavros_controllers::FlatTarget &msg);

  /**
   * @brief Yaw target callback
   * @param msg Yaw angle message
   */
  void yawtargetCallback(const std_msgs::Float32 &msg);

  /**
   * @brief Multi-DOF joint trajectory callback
   * @param msg Trajectory message
   */
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);

  /**
   * @brief Command loop callback
   * @param event Timer event
   */
  void cmdloopCallback(const ros::TimerEvent &event);

  /**
   * @brief MAV state callback
   * @param msg MAV state message
   */
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);

  /**
   * @brief MAV pose callback
   * @param msg Pose message
   */
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);

  /**
   * @brief MAV velocity callback
   * @param msg Velocity message
   */
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);

  /**
   * @brief Status loop callback
   * @param event Timer event
   */
  void statusloopCallback(const ros::TimerEvent &event);

  /**
   * @brief Control trigger callback
   * @return Service call result
   */
  bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * @brief Landing service callback
   * @return Service call result
   */
  bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

  /**
   * @brief Reset integrator
   */
  void resetIntegral();

  /**
   * @brief Convert vectors to pose stamped message
   * @param position Position vector
   * @param orientation Attitude quaternion
   * @return Pose stamped message
   */
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);

  /**
   * @brief Compute body rate commands
   * @param bodyrate_cmd Angular rate command output
   * @param target_acc Target acceleration
   */
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);

  /**
   * @brief Position controller
   * @param target_pos Target position
   * @param target_vel Target velocity
   * @param target_acc Target acceleration
   * @return Control output
   */
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                const Eigen::Vector3d &target_acc);

  /**
   * @brief PD position controller
   * @param pos_error Position error
   * @param vel_error Velocity error
   * @return Control output
   */
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);

  // Flight state enumeration
  enum FlightState {
    WAITING_FOR_HOME_POSE,  // Waiting for home position
    MISSION_EXECUTION,      // Mission in execution
    LANDING,               // Landing in progress
    LANDED                 // Landed
  } node_state;

  /**
   * @brief Wait for predicate to be satisfied
   * @param pred Predicate pointer
   * @param msg Wait message
   * @param hz Check frequency
   */
  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };

  // Other member variables
  geometry_msgs::Pose home_pose_;         // Home position
  bool received_home_pose;                // Home position received flag
  std::shared_ptr<Control> controller_;   // Controller pointer

 public:
  /**
   * @brief Dynamic parameter reconfigure callback
   * @param config Configuration parameters
   * @param level Reconfigure level
   */
  void dynamicReconfigureCallback(mavros_controllers::GeometricControllerConfig &config, uint32_t level);

  /**
   * @brief Constructor
   * @param nh Public node handle
   * @param nh_private Private node handle
   */
  MavrosControllers(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  /**
   * @brief Virtual destructor
   */
  virtual ~MavrosControllers();

  /**
   * @brief Get current states
   * @param pos Position vector
   * @param att Attitude quaternion
   * @param vel Velocity vector
   * @param angvel Angular velocity vector
   */
  void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
    pos = mavPos_;
    att = mavAtt_;
    vel = mavVel_;
    angvel = mavRate_;
  };

  /**
   * @brief Get control errors
   * @param pos Position error
   * @param vel Velocity error
   */
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
    pos = mavPos_ - targetPos_;
    vel = mavVel_ - targetVel_;
  };

  /**
   * @brief Set body rate command
   * @param bodyrate_command Angular rate command
   */
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };

  /**
   * @brief Set feedthrough enable state
   * @param feed_through Enable flag
   */
  void setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; };

  /**
   * @brief Set desired acceleration
   * @param acceleration Acceleration vector
   */
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };

  /**
   * @brief Convert acceleration to quaternion
   * @param vector_acc Acceleration vector
   * @param yaw Yaw angle
   * @return Attitude quaternion
   */
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

  /**
   * @brief Calculate yaw angle from velocity direction
   * @param velocity Velocity vector
   * @return Yaw angle
   */
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif
