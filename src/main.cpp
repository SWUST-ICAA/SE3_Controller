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
 * @brief MAVROS Controller Node
 *
 * This ROS node implements a comprehensive UAV control system that includes:
 * - Multiple control strategies (Geometric, Attitude, Jerk Tracking)
 * - Dynamic parameter reconfiguration
 * - Position and attitude control
 * - Trajectory tracking
 * 
 * The node performs these key steps:
 * 1. Initializes ROS communication
 * 2. Creates controller instance with public and private parameter spaces
 * 3. Sets up dynamic reconfiguration server
 * 4. Runs the main ROS event loop
 *
 * @author Nanwan <nanwan2004@126.com>
 */

#include "mavros_controllers/controller_manager.h"

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "mavros_controller");

    // Create node handles for public and private namespaces
    ros::NodeHandle nh("");          // Public namespace
    ros::NodeHandle nh_private("~"); // Private namespace

    // Initialize controller with both namespaces
    // This sets up all subscribers, publishers and services
    MavrosControllers* Controller = new MavrosControllers(nh, nh_private);

    // Set up dynamic reconfigure server
    // This allows runtime parameter updates
    dynamic_reconfigure::Server<mavros_controllers::GeometricControllerConfig> srv;
    dynamic_reconfigure::Server<mavros_controllers::GeometricControllerConfig>::CallbackType f;
    f = boost::bind(&MavrosControllers::dynamicReconfigureCallback, Controller, _1, _2);
    srv.setCallback(f);

    // Enter ROS event loop
    ros::spin();
    
    return 0;
}
