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
 * @file nonlinear_geometric_control.cpp
 * @brief Implementation of geometric tracking control on SE(3)
 *
 * This file implements a geometric tracking controller for quadrotor UAVs
 * based on the special Euclidean group SE(3). The controller provides
 * robust tracking performance by directly working with the nonlinear
 * geometric structure of the vehicle's configuration space.
 *
 * Key features:
 * - Geometric formulation using SE(3) and SO(3) groups
 * - Quaternion-based attitude representation
 * - Direct computation of body rates from attitude error
 * - Singularity-free attitude control
 *
 * Reference:
 * Lee, T., Leok, M., & McClamroch, N. H. (2010). Geometric tracking control
 * of a quadrotor UAV on SE(3). 49th IEEE Conference on Decision and Control.
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#include "mavros_controllers/nonlinear_geometric_control.h"

/**
 * @brief Constructor for NonlinearGeometricControl
 * @param attctrl_tau Attitude control time constant [s]
 * 
 * Initializes the geometric controller with a specified time constant
 * that determines the speed of attitude convergence.
 */
NonlinearGeometricControl::NonlinearGeometricControl(double attctrl_tau) : Control() {
    attctrl_tau_ = attctrl_tau;
}

/**
 * @brief Default destructor
 */
NonlinearGeometricControl::~NonlinearGeometricControl() {}

/**
 * @brief Update controller states and compute control commands
 * 
 * This implementation follows the geometric tracking control approach from Lee et al.
 * The controller operates directly on the special Euclidean group SE(3), which is
 * the configuration space of the quadrotor. This allows for globally defined,
 * singularity-free control of both position and attitude.
 *
 * Algorithm steps:
 * 1. Quaternion to Rotation Matrix Conversion
 *    - Convert current and reference quaternions to SO(3) rotation matrices
 *    - Ensures proper handling of attitude representation
 *
 * 2. Geometric Attitude Error Computation
 *    - Uses the Lie algebra so(3) formulation
 *    - Computes error as ea = vee(Rd^T * R - R^T * Rd)/2
 *    - This formulation ensures coordinate-free global stability
 *
 * 3. Body Rate Command Generation
 *    - Maps attitude error to desired body rates
 *    - Uses proportional control with time constant attctrl_tau_
 *    - Provides smooth convergence to desired attitude
 *
 * 4. Thrust Computation
 *    - Projects reference acceleration onto body z-axis
 *    - Ensures proper tracking of translational motion
 *
 * Note: This implementation generates angular rate commands instead of
 * moment commands as in the original paper, making it compatible with
 * typical flight controller offboard control interfaces.
 *
 * @param curr_att Current attitude quaternion [w,x,y,z]
 * @param ref_att Reference attitude quaternion [w,x,y,z]
 * @param ref_acc Reference acceleration vector [m/s^2]
 * @param ref_jerk Reference jerk vector [m/s^3] (unused in this controller)
 */
void NonlinearGeometricControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                     const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
    // Step 1: Convert quaternions to rotation matrices
    // These matrices belong to SO(3), the special orthogonal group
    Eigen::Matrix3d rotmat = quat2RotMatrix(curr_att);     // Current attitude
    Eigen::Matrix3d rotmat_d = quat2RotMatrix(ref_att);    // Desired attitude

    // Step 2: Compute attitude error on SE(3)
    // Using the formulation: ea = vee(Rd^T * R - R^T * Rd)
    // where vee is the inverse of the hat map (matrix_hat_inv)
    Eigen::Vector3d error_att = 0.5 * matrix_hat_inv(
        rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d
    );

    // Step 3: Convert error to desired body rates
    // Using proportional control with time constant tau
    desired_rate_ = (2.0 / attctrl_tau_) * error_att;

    // Step 4: Compute thrust command
    // Extract body z-axis for thrust projection
    const Eigen::Vector3d zb = rotmat.col(2);
    
    // Set thrust command
    desired_thrust_(0) = 0.0;             // No lateral thrust
    desired_thrust_(1) = 0.0;             // No lateral thrust
    desired_thrust_(2) = ref_acc.dot(zb); // Vertical thrust from acceleration projection
}
