# SE3 Geometric Controller

基于微分平坦性的SE(3)几何无人机控制器。

## 功能特点

- 基于微分平坦性的几何控制
- 支持多种控制模式：
  - 非线性几何控制
  - 非线性姿态控制
  - Jerk跟踪控制
- 自动起飞和降落功能
- 动态参数重配置
- 轨迹跟踪与位置控制

## 快速使用

### 启动控制器
```bash
roslaunch geometric_controller geometric_controller.launch
```

### 降落指令
```bash
rosservice call /land "data: true"
```

## 控制参数

主要控制参数在`config/geometric_controller_config.yaml`中配置：

- `Kp_x, Kp_y, Kp_z`: 位置控制增益
- `Kv_x, Kv_y, Kv_z`: 速度控制增益
- `attctrl_constant`: 姿态控制时间常数

## 话题接口

### 订阅
- `mavros/local_position/pose`: 当前位置和姿态
- `mavros/local_position/velocity_local`: 当前速度
- `mavros/state`: 飞控状态

### 发布
- `command/bodyrate_command`: 体框角速度指令
- `reference/pose`: 参考位姿

## 作者

- **Nanwan** - nanwan2004@126.com
