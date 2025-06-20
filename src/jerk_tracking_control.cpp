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
 * @file jerk_tracking_control.cpp
 * @brief Implementation of jerk-based trajectory tracking control
 *
 * This file implements a trajectory tracking controller that explicitly
 * considers jerk (derivative of acceleration) for improved tracking
 * performance. The controller combines feedforward and feedback terms
 * to achieve precise trajectory following while maintaining smooth motion.
 *
 * Key features:
 * - Explicit jerk feedforward control
 * - Geometric projection for consistent motion
 * - Quaternion-based attitude control
 * - Smooth trajectory tracking
 *
 * Reference:
 * Lopez, Brett Thomas. "Low-latency trajectory planning for high-speed navigation 
 * in unknown environments." MIT Dissertation, 2016.
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#include "mavros_controllers/jerk_tracking_control.h"

/**
 * @brief Constructor for JerkTrackingControl
 * 
 * Initializes the jerk tracking controller with default parameters.
 * The controller uses a fixed timestep of 0.01s (100Hz) for numerical
 * differentiation and integration.
 */
JerkTrackingControl::JerkTrackingControl() : Control() {}

/**
 * @brief Default destructor
 */
JerkTrackingControl::~JerkTrackingControl() {}

/**
 * @brief Update controller states and compute control commands
 * 
 * Implements a jerk-based trajectory tracking approach that combines
 * feedforward and feedback control. The algorithm uses geometric
 * projections and quaternion algebra to ensure consistent motion
 * while following the desired trajectory.
 *
 * Algorithm steps:
 * 1. Jerk Computation
 *    - Calculates feedback jerk through numerical differentiation
 *    - Combines with feedforward jerk term from trajectory
 *    - Uses fixed timestep dt = 0.01s for consistent behavior
 *
 * 2. Geometric Projection
 *    - Projects combined jerk onto plane perpendicular to acceleration
 *    - Ensures motion consistency by maintaining acceleration direction
 *    - Normalizes by acceleration magnitude for proper scaling
 *
 * 3. Quaternion Rotation
 *    - Converts geometric jerk to quaternion format
 *    - Computes attitude error quaternion
 *    - Performs frame transformation using quaternion algebra
 *
 * 4. Body Rate Computation
 *    - Extracts body rates from transformed jerk
 *    - Applies coordinate transformation to match body frame
 *    - Sets zero yaw rate for heading stability
 *
 * 5. Thrust Computation
 *    - Projects reference acceleration onto body z-axis
 *    - Computes vertical thrust command
 *    - Maintains zero lateral thrust components
 *
 * Mathematical formulation:
 * - Feedback jerk: j_fb = (a_ref - a_last) / dt
 * - Combined jerk: j_des = j_ref + j_fb
 * - Projected jerk: j_proj = j_des/|a| - a * (a·j_des)/|a|^3
 * - Body rates derived through quaternion rotations
 *
 * @param curr_att Current attitude quaternion [w,x,y,z]
 * @param ref_att Reference attitude quaternion [w,x,y,z]
 * @param ref_acc Reference acceleration vector [m/s^2]
 * @param ref_jerk Reference jerk vector [m/s^3]
 */
void JerkTrackingControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
    // Control loop timestep
    const double dt_ = 0.01;  // 100Hz control rate

    // Step 1: Compute feedback jerk using numerical differentiation
    const Eigen::Vector3d jerk_fb = (ref_acc - last_ref_acc_) / dt_;
    
    // Step 2: Combine feedforward and feedback jerk terms
    const Eigen::Vector3d jerk_des = ref_jerk + jerk_fb;
    
    // Get current body z-axis for thrust computation
    const Eigen::Matrix3d R = quat2RotMatrix(curr_att);
    const Eigen::Vector3d zb = R.col(2);

    // Step 3: Project jerk onto plane perpendicular to acceleration
    // This ensures the jerk component maintains the desired acceleration direction
    const Eigen::Vector3d jerk_vector =
        jerk_des / ref_acc.norm() - ref_acc * ref_acc.dot(jerk_des) / std::pow(ref_acc.norm(), 3);
    
    // Convert to quaternion format for rotation computations
    const Eigen::Vector4d jerk_vector4d(0.0, jerk_vector(0), jerk_vector(1), jerk_vector(2));

    // Step 4: Convert geometric jerk to body rates
    // First compute attitude error quaternion
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qd = quatMultiplication(q_inv, ref_att);

    // Compute conjugate for quaternion rotation
    const Eigen::Vector4d qd_star(qd(0), -qd(1), -qd(2), -qd(3));

    // Transform jerk to body frame
    const Eigen::Vector4d ratecmd_pre = quatMultiplication(quatMultiplication(qd_star, jerk_vector4d), qd);

    // Extract and assign body rates
    // Note: Coordinate transformation from geometric to body frame
    desired_rate_(0) = ratecmd_pre(2);    // Roll rate
    desired_rate_(1) = -ratecmd_pre(1);   // Pitch rate
    desired_rate_(2) = 0.0;               // Zero yaw rate command

    // Step 5: Compute thrust command
    desired_thrust_(0) = 0.0;             // No lateral thrust
    desired_thrust_(1) = 0.0;             // No lateral thrust
    desired_thrust_(2) = ref_acc.dot(zb); // Vertical thrust from acceleration projection

    // Store acceleration for next iteration
    last_ref_acc_ = ref_acc;
}
