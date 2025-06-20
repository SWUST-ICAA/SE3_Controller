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
 * @file nonlinear_attitude_control.cpp
 * @brief Implementation of nonlinear quaternion-based attitude control
 *
 * This file implements a nonlinear attitude controller for quadrotor UAVs
 * using quaternion feedback. The controller provides robust and singularity-free
 * attitude tracking by working directly with quaternion representations.
 *
 * Key features:
 * - Direct quaternion error feedback
 * - Nonlinear control law for improved performance
 * - Singularity-free attitude representation
 * - Smooth and stable convergence behavior
 *
 * Reference:
 * Brescianini, D., Hehn, M., & D'Andrea, R. (2013). Nonlinear quadrocopter 
 * attitude control: Technical report. ETH Zurich.
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#include "mavros_controllers/nonlinear_attitude_control.h"

/**
 * @brief Constructor for NonlinearAttitudeControl
 * @param attctrl_tau Attitude control time constant [s]
 * 
 * Initializes the attitude controller with a specified time constant
 * that determines the rate of attitude convergence.
 */
NonlinearAttitudeControl::NonlinearAttitudeControl(double attctrl_tau) : Control() {
    attctrl_tau_ = attctrl_tau;
}

/**
 * @brief Default destructor
 */
NonlinearAttitudeControl::~NonlinearAttitudeControl() {}

/**
 * @brief Update controller states and compute control commands
 * 
 * This implementation follows the nonlinear quaternion-based control approach
 * from Brescianini et al. The controller works directly with quaternion 
 * representations to avoid singularities and provide globally stable attitude
 * tracking.
 *
 * Algorithm steps:
 * 1. Quaternion Error Computation
 *    - Computes inverse of current quaternion
 *    - Performs quaternion multiplication: q_error = q_current^(-1) * q_reference
 *    - This represents the relative rotation needed to align current with reference
 *
 * 2. Nonlinear Feedback Control
 *    - Maps quaternion error to body rates using nonlinear feedback
 *    - Uses time constant attctrl_tau_ to tune convergence speed
 *    - Includes sign correction to ensure shortest-path rotation
 *    - Separate control for roll, pitch and yaw axes
 *
 * 3. Thrust Vector Computation
 *    - Projects reference acceleration onto current body z-axis
 *    - Ensures proper tracking of translational motion
 *    - Only vertical thrust is commanded (no lateral thrust)
 *
 * Mathematical formulation:
 * - Quaternion error: qe = q^(-1) * q_d
 * - Body rates: w = (2/tau) * sign(qe_w) * qe_v
 * where qe_w is scalar part and qe_v is vector part of error quaternion
 *
 * Note: The implementation uses the convention [w,x,y,z] for quaternions,
 * where w is the scalar component and [x,y,z] is the vector component.
 *
 * @param curr_att Current attitude quaternion [w,x,y,z]
 * @param ref_att Reference attitude quaternion [w,x,y,z]
 * @param ref_acc Reference acceleration vector [m/s^2]
 * @param ref_jerk Reference jerk vector [m/s^3] (unused in this controller)
 */
void NonlinearAttitudeControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                    const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
    // Step 1: Compute attitude error quaternion
    // Convert current quaternion to its inverse for error computation
    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    
    // Quaternion error = q_inv * q_ref (quaternion multiplication)
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);

    // Step 2: Compute desired body rates
    // Using nonlinear feedback law from Brescianini et al.
    // The 2/tau factor determines the convergence rate
    // copysign(1.0, qe(0)) ensures proper rotation direction
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);  // Roll rate
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);  // Pitch rate
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);  // Yaw rate

    // Step 3: Compute thrust command
    // Extract body z-axis for thrust projection
    const Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);
    const Eigen::Vector3d zb = rotmat.col(2);
    
    // Set thrust command
    desired_thrust_(0) = 0.0;             // No lateral thrust
    desired_thrust_(1) = 0.0;             // No lateral thrust
    desired_thrust_(2) = ref_acc.dot(zb); // Vertical thrust from acceleration projection
}
