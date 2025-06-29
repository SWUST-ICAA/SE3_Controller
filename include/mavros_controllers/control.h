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
 * @brief Abstract base class for all controller implementations
 *
 * This class defines the interface for different control strategies.
 * It provides virtual methods for updating controller states and 
 * accessing control outputs (desired thrust and angular rates).
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <Eigen/Dense>

class Control {
 public:
  /**
   * @brief Default constructor
   */
  Control() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~Control() = default;

  /**
   * @brief Updates the controller state
   * 
   * @param curr_att Current attitude quaternion [w,x,y,z]
   * @param ref_att Reference attitude quaternion [w,x,y,z]
   * @param ref_acc Reference acceleration vector [m/s^2]
   * @param ref_jerk Reference jerk vector [m/s^3]
   */
  virtual void Update(Eigen::Vector4d &curr_att, 
                     const Eigen::Vector4d &ref_att, 
                     const Eigen::Vector3d &ref_acc,
                     const Eigen::Vector3d &ref_jerk) = 0;

  /**
   * @brief Gets the desired thrust vector
   * @return Vector3d Desired thrust in body frame [N]
   */
  Eigen::Vector3d getDesiredThrust() { return desired_thrust_; }

  /**
   * @brief Gets the desired angular rates
   * @return Vector3d Desired angular rates [rad/s]
   */
  Eigen::Vector3d getDesiredRate() { return desired_rate_; }

 protected:
  Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};    ///< Desired angular rates
  Eigen::Vector3d desired_thrust_{Eigen::Vector3d::Zero()};  ///< Desired thrust vector

 private:
};

#endif
