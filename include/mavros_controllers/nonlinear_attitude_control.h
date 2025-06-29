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
 * @brief Nonlinear attitude controller implementation
 *
 * This class implements a nonlinear control approach specifically for
 * quadrotor attitude control. It inherits from the Control base class
 * and provides implementation for direct attitude tracking control.
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#ifndef NONLINEAR_ATTITUDE_CONTROL_H
#define NONLINEAR_ATTITUDE_CONTROL_H

#include "mavros_controllers/common.h"
#include "mavros_controllers/control.h"

class NonlinearAttitudeControl : public Control {
 public:
  /**
   * @brief Constructor
   * @param attctrl_tau Attitude control time constant [s]
   */
  explicit NonlinearAttitudeControl(double attctrl_tau);

  /**
   * @brief Destructor
   */
  virtual ~NonlinearAttitudeControl();

  /**
   * @brief Updates the attitude controller state
   * 
   * Implements a nonlinear control law for direct attitude tracking,
   * using quaternion-based attitude representation for improved stability
   * and performance.
   *
   * @param curr_att Current attitude quaternion [w,x,y,z]
   * @param ref_att Reference attitude quaternion [w,x,y,z]
   * @param ref_acc Reference acceleration vector [m/s^2]
   * @param ref_jerk Reference jerk vector [m/s^3]
   */
  void Update(Eigen::Vector4d &curr_att,
              const Eigen::Vector4d &ref_att,
              const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk) override;

 private:
  double attctrl_tau_{1.0};  ///< Attitude control time constant [s]
};

#endif
