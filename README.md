# Geometric Controller

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Version](https://img.shields.io/badge/version-v1.2.0-green.svg)](https://github.com/your-repo/geometric_controller)

SE(3) geometric controller for UAVs using differential flatness-based trajectory tracking.

## Features

- Three control modes: Quaternion-based, SE(3) geometric, and jerk tracking
- Real-time parameter tuning via dynamic reconfigure
- Automatic takeoff and landing

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/SWUST-ICAA/mavros_controllers.git
cd .. && catkin_make
```

## Usage

```bash
# Basic launch
roslaunch geometric_controller geometric_controller.launch


# Landing service
rosservice call /land "data: true"
```

## Configuration

Select controller type in `config/geometric_controller_config.yaml`:
- `1`: Quaternion-based attitude control
- `2`: SE(3) geometric control (default)
- `3`: Jerk feedforward control

## Topics

### Subscribed
- `/mavros/local_position/pose` - Current pose from flight controller
- `/mavros/local_position/velocity_local` - Current velocity in local frame
- `/mavros/state` - Flight controller state (armed, mode, etc.)
- `/reference/flatsetpoint` - Flat trajectory reference (position, velocity, acceleration)
- `/command/trajectory` - Multi-DOF trajectory commands
- `/reference/setpoint` - Simple position/velocity setpoint
- `/reference/yaw` - Yaw angle reference

### Published
- `/mavros/setpoint_raw/attitude` - Body rate and thrust commands to flight controller
- `/reference/pose` - Current reference pose for visualization
- `/geometric_controller/path` - Trajectory history for visualization
- `/mavros/companion_process/status` - Controller status
- `/mavros/setpoint_position/local` - Position setpoint for landing

## References

- Lee, T., Leok, M., & McClamroch, N. H. (2010). Geometric tracking control of a quadrotor UAV on SE(3). CDC.
- Faessler, M., et al. (2017). Differential flatness of quadrotor dynamics subject to rotor drag. IEEE RA-L.

## Author

**Nanwan** - nanwan2004@126.com

## License

BSD-3-Clause
