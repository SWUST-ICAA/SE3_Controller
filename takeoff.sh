#!/bin/bash

echo "等待MAVROS启动..."
sleep 2

echo "切换到OFFBOARD模式..."
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

sleep 2

echo "解锁飞机..."
rosservice call /mavros/cmd/arming "value: true"

sleep 2

echo "发送起飞命令..."
rostopic pub /reference/setpoint geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" -r 10
