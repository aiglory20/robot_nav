# 自动导航功能使用说明

## 概述

该功能实现了在仿真环境中自动导航到预定义的三个点 (A → B → C)。

## 使用步骤

### 1. 启动仿真环境

在第一个终端中,启动 Gazebo 仿真器:

```bash
cd ~/ros_ws
source install/setup.zsh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

**注意**: 需要点击 Gazebo 左下角橙红色的 `启动` 按钮

### 2. 启动导航系统

在第二个终端中,启动导航系统:

```bash
cd ~/ros_ws
source install/setup.zsh
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=False
```

等待导航系统完全启动 (RViz 窗口打开,机器人模型加载完成)。

### 3. 启动自动导航

在第三个终端中,启动自动导航节点:

```bash
cd ~/ros_ws
source install/setup.zsh
ros2 launch pb2025_nav_bringup auto_navigation_launch.py
```

机器人将自动依次导航到三个预定义的点。

## 自定义导航点

如果需要修改导航点的坐标,请编辑 `auto_navigation.py` 文件中的 `waypoints` 列表:

```python
self.waypoints = [
    {'name': 'Point A', 'x': 2.0, 'y': 2.0, 'yaw': 0.0},
    {'name': 'Point B', 'x': 6.0, 'y': 4.0, 'yaw': 1.57},
    {'name': 'Point C', 'x': 4.0, 'y': 6.0, 'yaw': 3.14},
]
```

- `x`, `y`: 在地图坐标系中的位置 (单位: 米)
- `yaw`: 目标朝向角度 (单位: 弧度, 0 表示向右, 1.57 表示向上, 3.14 表示向左, -1.57 表示向下)

### 如何获取合适的坐标

#### 方法1: 使用坐标获取工具 (推荐)

启动导航系统后,在新终端运行坐标获取工具:

```bash
cd ~/ros_ws
source install/setup.zsh
ros2 run pb2025_nav_bringup get_coordinates.py
```

然后在 RViz 中:

1. 使用 **"Nav2 Goal"** (2D Nav Goal) 工具点击目标位置
2. 或使用 **"2D Pose Estimate"** 工具点击任意位置
3. 终端会自动显示坐标,并给出可直接复制的代码格式

**示例输出:**

```
--- 航点 1 (Nav2 Goal) ---
  X:   2.500 m
  Y:   3.200 m
  Yaw: 1.571 rad (90.0°)

复制到代码中:
  {'name': 'Point A', 'x': 2.50, 'y': 3.20, 'yaw': 1.57},
```

#### 方法2: 手动订阅话题

使用 `Nav2 Goal` 工具点击目标点,然后在终端查看:

```bash
ros2 topic echo /red_standard_robot1/goal_pose
```

或使用 `2D Pose Estimate` 工具:

```bash
ros2 topic echo /initialpose
```

#### 方法3: 在 RViz 中查看

在 RViz 的底部状态栏,移动鼠标时会显示当前鼠标位置的坐标。

### 坐标系说明

- **X轴**: 向前为正
- **Y轴**: 向左为正
- **Yaw角度**:
  - `0` 或 `0.0` rad = 0° (向右)
  - `1.57` rad ≈ 90° (向前)
  - `3.14` rad ≈ 180° (向左)
  - `-1.57` rad ≈ -90° 或 270° (向后)

## 一键启动脚本 (可选)

如果想简化启动流程,可以创建一个启动脚本。这里提供了一个完整的启动示例,但需要手动在不同终端执行上述三个命令,因为它们需要顺序启动且保持运行。

## 故障排查

1. **目标被拒绝**: 确保导航系统已完全启动,可以在 RViz 中看到机器人和地图
2. **机器人不移动**: 检查是否点击了 Gazebo 的启动按钮
3. **路径规划失败**: 调整航点位置,确保它们在可通行区域内

## 参数说明

- `namespace`: 机器人命名空间,默认为 `red_standard_robot1`
- `use_sim_time`: 是否使用仿真时间,默认为 `true`

## 控制小车自旋转

### 关于小车自旋转的说明

在导航过程中,小车可能会进行自旋转(原地旋转)。这个行为受多个参数控制:

### 1. 关闭路径跟踪时的自旋转

在 `nav2_params.yaml` 的 `controller_server` -> `FollowPath` 部分:

```yaml
FollowPath:
  plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
  enable_rotation: false          # 设置为 false 关闭自旋转
  use_rotate_to_heading: false    # 设置为 false 关闭"先转向再前进"的行为
```

**参数说明:**

- `enable_rotation: false` - 禁用到达目标点时的自旋转对齐
- `use_rotate_to_heading: false` - 禁用路径跟踪前的预旋转

### 2. 关闭目标点朝向检查

在 `controller_server` -> `general_goal_checker` 部分:

```yaml
general_goal_checker:
  stateful: True
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.15
  yaw_goal_tolerance: 6.28    # 设置为 6.28 (2*pi) 表示不检查朝向
```

**说明:** `yaw_goal_tolerance: 6.28` 意味着任意朝向都可接受,机器人不会为了对齐朝向而旋转。

### 3. 云台自旋模式

如果您看到机器人在导航时云台持续自旋,这是由 `fake_vel_transform` 模块控制的:

```yaml
fake_vel_transform:
  ros__parameters:
    init_spin_speed: 3.14    # 云台自旋速度 (rad/s)
```

要停止云台自旋,可以发布话题:

```bash
# 停止自旋
ros2 topic pub --once /red_standard_robot1/cmd_spin std_msgs/msg/Float32 "{data: 0.0}"

# 恢复自旋 (速度: 3.14 rad/s)
ros2 topic pub --once /red_standard_robot1/cmd_spin std_msgs/msg/Float32 "{data: 3.14}"
```

### 4. 当前配置状态

查看您的配置文件 `config/simulation/nav2_params.yaml`,当前设置为:

```yaml
controller_server:
  FollowPath:
    enable_rotation: false           # ✅ 已关闭
    use_rotate_to_heading: false     # ✅ 已关闭

general_goal_checker:
  yaw_goal_tolerance: 6.28           # ✅ 不检查朝向
```

**结论:** 您的配置已经禁用了导航过程中的自旋转行为。如果仍然看到旋转,可能是云台自旋,使用上述命令停止。

### 修改配置后需要

```bash
cd ~/ros_ws
colcon build --packages-select pb2025_nav_bringup --symlink-install
source install/setup.zsh
# 重新启动导航系统
```
