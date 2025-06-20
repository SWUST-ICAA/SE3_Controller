/****************************************************************************
 * Copyright (c) 2025 Nanwan. All rights reserved.
 *
 * This software is licensed under the terms of the BSD-3-Clause license.
 * Please see LICENSE file in the root directory for details.
 ****************************************************************************/

/**
 * @file controller_manager.cpp
 * @brief Controller Manager Implementation
 * 
 * This file implements a controller manager for MAV control that supports multiple
 * control strategies including geometric control, attitude control and trajectory tracking.
 * It handles controller initialization, state management, command processing and 
 * feedback control computation.
 *
 * Key Features:
 * - Multiple controller types support (Attitude, Geometric, Jerk Tracking)
 * - Position and attitude control with drag compensation
 * - Automatic takeoff and landing capability
 * - Dynamic parameter reconfiguration
 * - State monitoring and safety checks
 *
 * @author Nanwan <nanwan2004@126.com>
 */

 #include "mavros_controllers/controller_manager.h"
 #include "mavros_controllers/jerk_tracking_control.h"
 #include "mavros_controllers/nonlinear_attitude_control.h"
 #include "mavros_controllers/nonlinear_geometric_control.h"
 
 using namespace Eigen;
 using namespace std;
 
 // System constants
 namespace {
     // Control loop rates
     constexpr double CONTROL_LOOP_RATE = 0.01;    // 100Hz control loop
     constexpr double STATUS_LOOP_RATE = 0.5;      // 2Hz status updates
     
     // Timing parameters
     constexpr double TAKEOFF_TIMEOUT = 3.0;       // Seconds to wait before enabling integral control
     constexpr double SERVICE_TIMEOUT = 0.5;       // Service call timeout [s]
     
     // System parameters
     constexpr int MAV_COMPANION_ID = 196;         // MAV_COMPONENT_ID_AVOIDANCE
     constexpr int DEFAULT_HISTORY_SIZE = 200;     // Default pose history window size
 }
 
 /**
  * @brief Constructor for MavrosControllers
  * 
  * Initializes the controller with ROS node handles and sets up:
  * - ROS subscribers for state and command inputs
  * - ROS publishers for control outputs
  * - Service servers for control triggering and landing
  * - Timer callbacks for control and status loops
  * - Controller parameters from ROS parameter server
  * - Controller type based on configuration
  *
  * @param nh ROS node handle for general topics
  * @param nh_private ROS node handle for private parameters
  */
 MavrosControllers::MavrosControllers(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
     : nh_(nh),
       nh_private_(nh_private),
       node_state(WAITING_FOR_HOME_POSE) {
     
     // Initialize subscribers
     referenceSub_ = nh_.subscribe("reference/setpoint", 1,
         &MavrosControllers::targetCallback, this, ros::TransportHints().tcpNoDelay());
     
     flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1,
         &MavrosControllers::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
     
     yawreferenceSub_ = nh_.subscribe("reference/yaw", 1,
         &MavrosControllers::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
     
     multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1,
         &MavrosControllers::multiDOFJointCallback, this, ros::TransportHints().tcpNoDelay());
     
     mavstateSub_ = nh_.subscribe("mavros/state", 1,
         &MavrosControllers::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
     
     mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1,
         &MavrosControllers::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
     
     mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1,
         &MavrosControllers::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());
 
     // Initialize publishers
     angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
         "mavros/setpoint_raw/attitude", 1);
     
     referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>(
         "reference/pose", 1);
     
     target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
         "mavros/setpoint_position/local", 10);
     
     posehistoryPub_ = nh_.advertise<nav_msgs::Path>(
         "mavros_controllers/path", 10);
     
     systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
         "mavros/companion_process/status", 1);
         
     // Initialize services
     ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller",
         &MavrosControllers::ctrltriggerCallback, this);
     
     land_service_ = nh_.advertiseService("land",
         &MavrosControllers::landCallback, this);
     
     arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
         "mavros/cmd/arming");
     
     set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
         "mavros/set_mode");
         
     // Initialize timers
     cmdloop_timer_ = nh_.createTimer(ros::Duration(CONTROL_LOOP_RATE),
         &MavrosControllers::cmdloopCallback, this);
     
     statusloop_timer_ = nh_.createTimer(ros::Duration(STATUS_LOOP_RATE),
         &MavrosControllers::statusloopCallback, this);
         
     // Load controller parameters
     int controller_type;
     double dx, dy, dz, attctrl_tau;
     
     nh_private_.param<int>("controller_type", controller_type, 2);
     nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
     nh_private_.param<bool>("auto_takeoff", auto_takeoff, true);
     nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
     nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
     nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
     
     // Load drag compensation parameters
     nh_private_.param<double>("drag_dx", dx, 0.0);
     nh_private_.param<double>("drag_dy", dy, 0.0);
     nh_private_.param<double>("drag_dz", dz, 0.0);
     D_ << dx, dy, dz;
     
     // Load control parameters
     nh_private_.param<double>("attctrl_constant", attctrl_tau, 0.1);
     nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);
     nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);
     
     // Load position control gains
     nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
     nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
     nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
     
     // Load velocity control gains
     nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
     nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
     nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
     
     // Load integral control parameters
     nh_private_.param<double>("Kint_x", Kint_x_, 0.0);
     nh_private_.param<double>("Kint_y", Kint_y_, 0.0);
     nh_private_.param<double>("Kint_z", Kint_z_, 0.0);
     nh_private_.param<double>("max_integral", max_int_, 2.0);
     nh_private_.param<bool>("enable_integral", enable_integral_, false);
     
     // Load system parameters
     nh_private_.param<int>("posehistory_window", posehistory_window_, DEFAULT_HISTORY_SIZE);
     nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
     nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
     nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);
 
     // Initialize controller based on type
     switch (controller_type) {
         case 1:  // ERROR_QUATERNION
             controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
             ctrl_mode_ = ERROR_QUATERNION;
             ROS_INFO("Initialized Nonlinear Attitude Controller (Quaternion Error)");
             break;
             
         case 2:  // ERROR_GEOMETRIC
             controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
             ctrl_mode_ = ERROR_GEOMETRIC;
             ROS_INFO("Initialized Nonlinear Geometric Controller (SE(3) Error)");
             break;
             
         case 3:  // JERK_TRACKING
             controller_ = std::make_shared<JerkTrackingControl>();
             ROS_INFO("Initialized Jerk Tracking Controller");
             break;
             
         default:
             ROS_WARN("Invalid controller_type: %d. Using default Geometric Controller", controller_type);
             controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
             ctrl_mode_ = ERROR_GEOMETRIC;
     }
 
     // Initialize state vectors
     targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
     targetVel_ << 0.0, 0.0, 0.0;
     mavPos_ << 0.0, 0.0, 0.0;
     mavVel_ << 0.0, 0.0, 0.0;
     pos_int_ << 0.0, 0.0, 0.0;
     start_time_ = ros::Time::now();
 
     // Initialize control gain matrices
     Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
     Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
     Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
 }
 
 /**
  * @brief Destructor for MavrosControllers
  * 
  * Cleans up resources and stops any running timers.
  */
 MavrosControllers::~MavrosControllers() {
     // Destructor implementation
 }
 
 /**
  * @brief Reset integral error terms to zero
  * 
  * This function resets the position integral error to zero vector.
  * Called when disabling integral control or during landing to prevent
  * integral windup issues.
  */
 void MavrosControllers::resetIntegral() {
     pos_int_ = Eigen::Vector3d::Zero();
 }
 
 /**
  * @brief Callback for target position and velocity setpoints
  * 
  * Processes target setpoint messages containing position and velocity data.
  * Updates target position, velocity and calculates target acceleration based
  * on numerical differentiation of velocity commands.
  * 
  * @param msg TwistStamped message containing position (angular) and velocity (linear) data
  */
 void MavrosControllers::targetCallback(const geometry_msgs::TwistStamped &msg) {
     // Store previous values for acceleration calculation
     reference_request_last_ = reference_request_now_;
     targetPos_prev_ = targetPos_;
     targetVel_prev_ = targetVel_;
 
     // Update timing information
     reference_request_now_ = ros::Time::now();
     reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
 
     // Update target states from message
     targetPos_ = toEigen(msg.twist.angular);  // Position encoded in angular field
     targetVel_ = toEigen(msg.twist.linear);   // Velocity encoded in linear field
 
     // Calculate target acceleration via numerical differentiation
     if (reference_request_dt_ > 0) {
         targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
     } else {
         targetAcc_ = Eigen::Vector3d::Zero();
     }
 }
 
 /**
  * @brief Callback for flat target setpoints
  * 
  * Processes flat target messages containing full state information
  * (position, velocity, acceleration). The type_mask field determines
  * which states are valid and should be used in the message.
  * 
  * @param msg FlatTarget message containing position, velocity and acceleration
  */
 void MavrosControllers::flattargetCallback(const mavros_controllers::FlatTarget &msg) {
     // Store previous values for continuity
     reference_request_last_ = reference_request_now_;
     targetPos_prev_ = targetPos_;
     targetVel_prev_ = targetVel_;
 
     // Update timing information
     reference_request_now_ = ros::Time::now();
     reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
 
     // Update position and velocity targets
     targetPos_ = toEigen(msg.position);
     targetVel_ = toEigen(msg.velocity);
 
     // Process acceleration based on type mask
     if (msg.type_mask == 1 || msg.type_mask == 2) {
         targetAcc_ = toEigen(msg.acceleration);  // Use provided acceleration
     } else if (msg.type_mask == 4) {
         targetAcc_ = Eigen::Vector3d::Zero();    // Zero acceleration commanded
     } else {
         targetAcc_ = toEigen(msg.acceleration);  // Default: use provided acceleration
     }
 }
 
 /**
  * @brief Callback for yaw setpoint commands
  * 
  * Updates the target yaw angle if velocity yaw mode is disabled.
  * In velocity yaw mode, yaw is computed from velocity direction.
  * 
  * @param msg Float32 message containing yaw angle in radians
  */
 void MavrosControllers::yawtargetCallback(const std_msgs::Float32 &msg) {
     if (!velocity_yaw_) {
         mavYaw_ = double(msg.data);
     }
 }
 
 /**
  * @brief Callback for multi-DOF joint trajectory messages
  * 
  * Processes trajectory messages containing complete state information including:
  * - Position and orientation from transforms
  * - Linear and angular velocities
  * - Linear and angular accelerations
  * Also extracts yaw angle from quaternion if velocity_yaw mode is disabled.
  * 
  * @param msg MultiDOFJointTrajectory message containing trajectory point data
  */
 void MavrosControllers::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
     trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
     
     // Store previous states for continuity
     reference_request_last_ = reference_request_now_;
     targetPos_prev_ = targetPos_;
     targetVel_prev_ = targetVel_;
 
     // Update timing information
     reference_request_now_ = ros::Time::now();
     reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();
 
     // Extract target states from trajectory point
     targetPos_ << pt.transforms[0].translation.x, 
                   pt.transforms[0].translation.y, 
                   pt.transforms[0].translation.z;
                   
     targetVel_ << pt.velocities[0].linear.x, 
                   pt.velocities[0].linear.y, 
                   pt.velocities[0].linear.z;
 
     targetAcc_ << pt.accelerations[0].linear.x, 
                   pt.accelerations[0].linear.y, 
                   pt.accelerations[0].linear.z;
 
     // Extract yaw from quaternion if not using velocity yaw
     if (!velocity_yaw_) {
         Eigen::Quaterniond q(pt.transforms[0].rotation.w,
                              pt.transforms[0].rotation.x,
                              pt.transforms[0].rotation.y,
                              pt.transforms[0].rotation.z);
         Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // Convert to RPY
         mavYaw_ = rpy(2);  // Extract yaw angle
     }
 }
 
 /**
  * @brief Callback for MAV pose updates
  * 
  * Processes local position and attitude information from the MAV.
  * Also handles initialization of home position on first received pose,
  * which is used as reference for landing operations.
  * 
  * @param msg PoseStamped message containing position and orientation data
  */
 void MavrosControllers::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
     // Initialize home pose on first message received
     if (!received_home_pose) {
         received_home_pose = true;
         home_pose_ = msg.pose;
         ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
     }
 
     // Update current MAV position and attitude
     mavPos_ = toEigen(msg.pose.position);
     mavAtt_ << msg.pose.orientation.w,  // Quaternion format: [w, x, y, z]
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z;
 }
 
 /**
  * @brief Callback for MAV velocity updates
  * 
  * Processes linear and angular velocity information from the MAV.
  * These velocities are used in the control feedback loops.
  * 
  * @param msg TwistStamped message containing linear and angular velocity
  */
 void MavrosControllers::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
     mavVel_ = toEigen(msg.twist.linear);   // Linear velocity in body frame
     mavRate_ = toEigen(msg.twist.angular); // Angular velocity in body frame
 }
 
 /**
  * @brief Callback for MAV state updates
  * 
  * Updates the stored MAV state with the latest information from
  * the flight control unit including flight mode and arming status.
  * 
  * @param msg State message containing flight mode, arming status, etc.
  */
 void MavrosControllers::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { 
     current_state_ = *msg; 
 }
 
 /**
  * @brief Service callback to trigger landing sequence
  * 
  * Sets the node state to LANDING when landing is requested.
  * The actual landing sequence is handled in the command loop callback.
  * 
  * @param request Service request (unused)
  * @param response Service response (unused)
  * @return true Always returns true to indicate successful callback execution
  */
 bool MavrosControllers::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
     node_state = LANDING;  // Trigger landing sequence
     return true;
 }
 
 /**
  * @brief Service callback to trigger controller mode changes
  * 
  * Allows external nodes to change the controller operating mode.
  * Used for switching between different control strategies.
  * 
  * @param req Service request containing new controller mode
  * @param res Service response containing success status
  * @return true Always returns true to indicate successful callback execution
  */
 bool MavrosControllers::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
     unsigned char mode = req.data;
 
     ctrl_mode_ = mode;
     res.success = ctrl_mode_;
     res.message = "controller triggered";
     return true;
 }
 
 /**
  * @brief Main control loop callback
  * 
  * This callback handles the main control logic based on the current node state:
  * 1. WAITING_FOR_HOME_POSE: Waits for initial home position
  * 2. MISSION_EXECUTION: Executes position control and publishes commands
  * 3. LANDING: Initiates and manages landing sequence
  * 4. LANDED: Stops control loop after successful landing
  * 
  * @param event Timer event containing timing information
  */
 void MavrosControllers::cmdloopCallback(const ros::TimerEvent &event) {
     switch (node_state) {
         case WAITING_FOR_HOME_POSE:
             waitForPredicate(&received_home_pose, "Waiting for home pose...");
             ROS_INFO("Got pose! Drone Ready to be armed.");
             node_state = MISSION_EXECUTION;
             break;
 
         case MISSION_EXECUTION: {
             // Compute desired acceleration
             Eigen::Vector3d desired_acc;
             if (feedthrough_enable_) {
                 desired_acc = targetAcc_;  // Direct feedthrough mode
             } else {
                 desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);  // Feedback control
             }
             
             // Compute body rate commands and publish
             computeBodyRateCmd(cmdBodyRate_, desired_acc);
             pubReferencePose(targetPos_, q_des);
             pubRateCommands(cmdBodyRate_, q_des);
             
             // Update and publish pose history for visualization
             appendPoseHistory();
             pubPoseHistory();
             break;
         }
 
         case LANDING: {
             // Execute landing sequence
             geometry_msgs::PoseStamped landingmsg;
             resetIntegral();  // Reset integral terms when landing
             
             landingmsg.header.stamp = ros::Time::now();
             landingmsg.pose = home_pose_;
             landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;  // Land 1m above home
             target_pose_pub_.publish(landingmsg);
             
             node_state = LANDED;
             ros::spinOnce();
             break;
         }
 
         case LANDED:
             ROS_INFO("Landed. Please set to position control and disarm.");
             cmdloop_timer_.stop();  // Stop control loop
             break;
     }
 }
 
 /**
  * @brief Status monitoring and auto-takeoff management loop
  * 
  * This callback manages:
  * - Automatic enabling of OFFBOARD mode
  * - Vehicle arming sequence
  * - System status publication to flight controller
  * 
  * Runs at a lower frequency than the control loop for efficient
  * system monitoring and state management.
  * 
  * @param event Timer event containing timing information
  */
 void MavrosControllers::statusloopCallback(const ros::TimerEvent &event) {
     if (auto_takeoff) {
         // Enable OFFBOARD mode and arm automatically
         // This will only run if the vehicle is simulated
         mavros_msgs::SetMode offb_set_mode;
         arm_cmd_.request.value = true;
         offb_set_mode.request.custom_mode = "OFFBOARD";
         
         // Request OFFBOARD mode if not already active
         if (current_state_.mode != "OFFBOARD" && 
             (ros::Time::now() - last_request_ > ros::Duration(0.5))) {
             if (set_mode_client_.call(offb_set_mode) && 
                 offb_set_mode.response.mode_sent) {
                 ROS_INFO("Offboard enabled");
             }
             last_request_ = ros::Time::now();
         } else {
             // Request arming if not already armed
             if (!current_state_.armed && 
                 (ros::Time::now() - last_request_ > ros::Duration(0.5))) {
                 if (arming_client_.call(arm_cmd_) && 
                     arm_cmd_.response.success) {
                     ROS_INFO("Vehicle armed");
                 }
                 last_request_ = ros::Time::now();
             }
         }
     }
     
     pubSystemStatus();  // Publish system status to flight controller
 }
 
 /**
  * @brief Publishes reference pose for visualization and logging
  * 
  * Converts the target position and attitude to a ROS pose message
  * and publishes it on the reference topic. Used for monitoring
  * the desired vehicle state and debugging control performance.
  * 
  * @param target_position Target position vector [x,y,z]
  * @param target_attitude Target attitude quaternion [w,x,y,z]
  */
 void MavrosControllers::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
     geometry_msgs::PoseStamped msg;
 
     msg.header.stamp = ros::Time::now();
     msg.header.frame_id = "map";
     msg.pose.position.x = target_position(0);
     msg.pose.position.y = target_position(1);
     msg.pose.position.z = target_position(2);
     msg.pose.orientation.w = target_attitude(0);
     msg.pose.orientation.x = target_attitude(1);
     msg.pose.orientation.y = target_attitude(2);
     msg.pose.orientation.z = target_attitude(3);
     referencePosePub_.publish(msg);
 }
 
 /**
  * @brief Publishes attitude rate commands to flight control unit
  * 
  * Formats and publishes angular rate and thrust commands to MAVROS.
  * The message includes:
  * - Body angular rates [roll_rate, pitch_rate, yaw_rate]
  * - Thrust command normalized to [0,1]
  * - Target attitude quaternion for reference
  * 
  * @param cmd Command vector [roll_rate, pitch_rate, yaw_rate, thrust]
  * @param target_attitude Target attitude quaternion [w,x,y,z]
  */
 void MavrosControllers::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
     mavros_msgs::AttitudeTarget msg;
 
     msg.header.stamp = ros::Time::now();
     msg.header.frame_id = "map";
     msg.body_rate.x = cmd(0);      // Roll rate
     msg.body_rate.y = cmd(1);      // Pitch rate
     msg.body_rate.z = cmd(2);      // Yaw rate
     msg.type_mask = 128;           // Ignore orientation messages, use rates only
     msg.orientation.w = target_attitude(0);
     msg.orientation.x = target_attitude(1);
     msg.orientation.y = target_attitude(2);
     msg.orientation.z = target_attitude(3);
     msg.thrust = cmd(3);           // Normalized thrust [0,1]
 
     angularVelPub_.publish(msg);
 }
 
 /**
  * @brief Publishes vehicle pose history for visualization
  * 
  * Creates and publishes a path message containing the vehicle's
  * recent pose history. Useful for trajectory visualization,
  * performance analysis, and debugging control behavior.
  */
 void MavrosControllers::pubPoseHistory() {
     nav_msgs::Path msg;
 
     msg.header.stamp = ros::Time::now();
     msg.header.frame_id = "map";
     msg.poses = posehistory_vector_;
 
     posehistoryPub_.publish(msg);
 }
 
 /**
  * @brief Publishes companion computer system status
  * 
  * Publishes the current state of the companion computer to MAVROS.
  * Used to indicate controller health and operation status to the
  * flight control system for monitoring and safety purposes.
  */
 void MavrosControllers::pubSystemStatus() {
     mavros_msgs::CompanionProcessStatus msg;
 
     msg.header.stamp = ros::Time::now();
     msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
     msg.state = (int)companion_state_;
 
     systemstatusPub_.publish(msg);
 }
 
 /**
  * @brief Updates the pose history buffer
  * 
  * Adds current pose to the history buffer and removes oldest entries
  * if buffer size exceeds the configured window size. Maintains a
  * fixed-size sliding window of recent vehicle poses for visualization.
  */
 void MavrosControllers::appendPoseHistory() {
     posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
     if (posehistory_vector_.size() > posehistory_window_) {
         posehistory_vector_.pop_back();
     }
 }
 
 /**
  * @brief Converts Eigen vectors to ROS pose message
  * 
  * Utility function that creates a PoseStamped message from position
  * and orientation vectors. Automatically sets the timestamp and
  * frame_id for consistency across the system.
  * 
  * @param position Position vector [x,y,z]
  * @param orientation Orientation quaternion [w,x,y,z]
  * @return Formatted PoseStamped message ready for publishing
  */
 geometry_msgs::PoseStamped MavrosControllers::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                       Eigen::Vector4d &orientation) {
     geometry_msgs::PoseStamped encode_msg;
     encode_msg.header.stamp = ros::Time::now();
     encode_msg.header.frame_id = "map";
     encode_msg.pose.orientation.w = orientation(0);
     encode_msg.pose.orientation.x = orientation(1);
     encode_msg.pose.orientation.y = orientation(2);
     encode_msg.pose.orientation.z = orientation(3);
     encode_msg.pose.position.x = position(0);
     encode_msg.pose.position.y = position(1);
     encode_msg.pose.position.z = position(2);
     return encode_msg;
 }
 
 /**
  * @brief Computes position control outputs using cascaded control structure
  * 
  * Implements a cascaded position control scheme based on differential flatness:
  * 1. Position and velocity error computation
  * 2. PID control with optional integral term and anti-windup
  * 3. Rotor drag compensation for improved tracking
  * 4. Feed-forward acceleration from trajectory
  * 
  * The controller compensates for rotor drag effects and includes
  * gravity compensation for stable hovering performance.
  * 
  * @param target_pos Target position [x,y,z] in meters
  * @param target_vel Target velocity [vx,vy,vz] in m/s
  * @param target_acc Target acceleration [ax,ay,az] in m/s^2
  * @return Desired acceleration command vector in m/s^2
  */
 Eigen::Vector3d MavrosControllers::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                    const Eigen::Vector3d &target_acc) {
     // Compute body rate commands using differential flatness
     // Controller based on Faessler 2017
     const Eigen::Vector3d a_ref = target_acc;
     
     // Update yaw based on velocity direction if enabled
     if (velocity_yaw_) {
         mavYaw_ = getVelocityYaw(mavVel_);
     }
 
     // Compute reference attitude from acceleration and yaw
     const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
     const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
 
     // Compute position and velocity errors
     const Eigen::Vector3d pos_error = mavPos_ - target_pos;
     const Eigen::Vector3d vel_error = mavVel_ - target_vel;
 
     // Position controller feedback
     const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);
 
     // Rotor drag compensation (velocity-dependent drag forces)
     const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;
 
     // Combine feedback, feedforward, drag compensation, and gravity
     const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;
 
     return a_des;
 }
 
 /**
  * @brief Computes body rate commands from desired acceleration
  * 
  * Converts desired acceleration to attitude and body rate commands using:
  * 1. Desired attitude computation from acceleration vector and yaw
  * 2. Controller update with current and desired states
  * 3. Body rate and thrust command extraction with normalization
  * 
  * @param bodyrate_cmd Output body rate command [p,q,r,thrust]
  * @param a_des Desired acceleration vector [m/s^2]
  */
 void MavrosControllers::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, 
                                            const Eigen::Vector3d &a_des) {
     // Compute reference attitude from desired acceleration
     q_des = acc2quaternion(a_des, mavYaw_);
 
     // Update controller with current and desired states
     controller_->Update(mavAtt_, q_des, a_des, Eigen::Vector3d::Zero());
     bodyrate_cmd.head(3) = controller_->getDesiredRate();
     
     // Calculate normalized thrust with proper scaling and limits
     double thrust_command = controller_->getDesiredThrust().z();
     double normalized_thrust = norm_thrust_const_ * thrust_command + norm_thrust_offset_;
     bodyrate_cmd(3) = std::max(0.0, std::min(1.0, normalized_thrust));  // Clamp to [0,1]
 }
 
 /**
  * @brief PID position control law implementation with anti-windup
  * 
  * Computes control output using PID feedback with the following features:
  * - Proportional term: Position error feedback
  * - Derivative term: Velocity error feedback  
  * - Integral term: Optional integral error feedback with anti-windup
  * - Delayed integral activation: Prevents windup during takeoff
  * - Acceleration command limiting: Prevents excessive control effort
  * 
  * @param pos_error Position error vector [m]
  * @param vel_error Velocity error vector [m/s]
  * @return Control acceleration vector [m/s^2]
  */
 Eigen::Vector3d MavrosControllers::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
     // Update integral error with anti-windup protection
     if (enable_integral_) {
         // Only integrate after 3 seconds from start to prevent takeoff windup
         double time_since_start = (ros::Time::now() - start_time_).toSec();
         if (time_since_start > 3.0) {
             pos_int_ += pos_error * 0.01;  // dt = 0.01 from cmdloop timer
             
             // Anti-windup: limit integral error magnitude
             for (int i = 0; i < 3; i++) {
                 pos_int_(i) = std::max(-max_int_, std::min(max_int_, pos_int_(i)));
             }
         } else {
             resetIntegral();  // Keep integral term zero during first 3 seconds
         }
     }
     
     // PID control law: a_fb = Kp*e_pos + Kd*e_vel + Ki*integral(e_pos)
     Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + 
                            Kvel_.asDiagonal() * vel_error + 
                            Kint_.asDiagonal() * pos_int_;
 
     // Saturate acceleration command to prevent excessive control effort
     if (a_fb.norm() > max_fb_acc_)
         a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;
 
     return a_fb;
 }
 
 /**
  * @brief Converts acceleration vector to attitude quaternion
  * 
  * Computes desired attitude quaternion from acceleration command and yaw angle.
  * Uses the fact that thrust direction defines the body z-axis, and yaw
  * defines rotation about that axis.
  * 
  * @param vector_acc Desired acceleration vector (defines thrust direction)
  * @param yaw Desired yaw angle in radians
  * @return Attitude quaternion [w,x,y,z]
  */
 Eigen::Vector4d MavrosControllers::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
     Eigen::Vector4d quat;
     Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
     Eigen::Matrix3d rotmat;
 
     // Project desired x-direction onto horizontal plane
     proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
 
     // Desired body z-axis from thrust direction
     zb_des = vector_acc / vector_acc.norm();
     
     // Desired body y-axis from cross product (right-hand rule)
     yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
     
     // Desired body x-axis completes the orthogonal frame
     xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
 
     // Construct rotation matrix and convert to quaternion
     rotmat << xb_des(0), yb_des(0), zb_des(0), 
               xb_des(1), yb_des(1), zb_des(1), 
               xb_des(2), yb_des(2), zb_des(2);
     quat = rot2Quaternion(rotmat);
     return quat;
 }
 
 /**
  * @brief Dynamic reconfiguration callback for online parameter tuning
  * 
  * Handles real-time parameter updates from the dynamic reconfigure server.
  * Allows online tuning of control gains and system parameters without
  * restarting the controller node.
  * 
  * @param config New configuration parameters
  * @param level Reconfiguration level (unused)
  */
 void MavrosControllers::dynamicReconfigureCallback(mavros_controllers::GeometricControllerConfig &config,
                                                    uint32_t level) {
     // Check and update each parameter if changed
     if (max_fb_acc_ != config.max_acc) {
         max_fb_acc_ = config.max_acc;
         ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
     } else if (Kpos_x_ != config.Kp_x) {
         Kpos_x_ = config.Kp_x;
         ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
     } else if (Kpos_y_ != config.Kp_y) {
         Kpos_y_ = config.Kp_y;
         ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
     } else if (Kpos_z_ != config.Kp_z) {
         Kpos_z_ = config.Kp_z;
         ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
     } else if (Kvel_x_ != config.Kv_x) {
         Kvel_x_ = config.Kv_x;
         ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
     } else if (Kvel_y_ != config.Kv_y) {
         Kvel_y_ = config.Kv_y;
         ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
     } else if (Kvel_z_ != config.Kv_z) {
         Kvel_z_ = config.Kv_z;
         ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
     } else if (Kint_x_ != config.Kint_x) {
         Kint_x_ = config.Kint_x;
         ROS_INFO("Reconfigure request : Kint_x = %.2f  ", config.Kint_x);
     } else if (Kint_y_ != config.Kint_y) {
         Kint_y_ = config.Kint_y;
         ROS_INFO("Reconfigure request : Kint_y = %.2f  ", config.Kint_y);
     } else if (Kint_z_ != config.Kint_z) {
         Kint_z_ = config.Kint_z;
         ROS_INFO("Reconfigure request : Kint_z = %.2f  ", config.Kint_z);
     } else if (max_int_ != config.max_integral) {
         max_int_ = config.max_integral;
         ROS_INFO("Reconfigure request : max_integral = %.2f  ", config.max_integral);
     } else if (enable_integral_ != config.enable_integral) {
         enable_integral_ = config.enable_integral;
         if (enable_integral_) {
             ROS_INFO("Integral control enabled");
             resetIntegral();  // Reset integral when enabling
         } else {
             ROS_INFO("Integral control disabled");
             resetIntegral();  // Reset integral when disabling
         }
     }
 
     // Update gain matrices with new values
     Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
     Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
     Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
 }