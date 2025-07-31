# API 文档

## 📚 概述

本文档详细介绍了力反馈主从控制系统的API接口，包括机器人模型、控制算法、运动学计算等核心模块。

## 🤖 机器人模型

### RobotInterface (基础接口)

所有机器人模型的基类，定义了统一的接口规范。

```python
from src.robot_models.robot_interface import RobotInterface, RobotType, RobotState

class RobotInterface:
    """机器人接口基类"""
    
    def __init__(self, robot_type: RobotType, robot_id: str):
        self.robot_type = robot_type
        self.robot_id = robot_id
        self.state = RobotState.DISCONNECTED
    
    def connect(self) -> bool:
        """连接机器人"""
        pass
    
    def disconnect(self) -> bool:
        """断开连接"""
        pass
    
    def get_joint_positions(self) -> np.ndarray:
        """获取关节位置"""
        pass
    
    def get_joint_velocities(self) -> np.ndarray:
        """获取关节速度"""
        pass
    
    def move_to_joint_positions(self, positions: np.ndarray, blocking: bool = True) -> bool:
        """移动到指定关节位置"""
        pass
    
    def get_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位姿"""
        pass
```

### D1ArmRobot (D1机械臂)

Unitree D1机械臂的完整实现。

```python
from src.robot_models.d1arm_robot import D1ArmRobot

# 创建D1机械臂实例
robot = D1ArmRobot(
    robot_id="d1_master",
    ip_address="192.168.123.161",
    port=8080
)

# 连接机械臂
if robot.connect():
    print("✅ D1机械臂连接成功")
    
    # 获取关节状态
    joint_pos = robot.get_joint_positions()
    joint_vel = robot.get_joint_velocities()
    
    # 移动到目标位置
    target_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.move_to_joint_positions(target_positions)
```

### URRobot (UR机械臂)

Universal Robots机械臂接口。

```python
from src.robot_models.ur_robot import URRobot, RobotType

# 创建UR机械臂实例
ur_robot = URRobot(
    robot_type=RobotType.UR3E,
    robot_id="ur3e_slave",
    node_name='ur3e_controller'
)

# 连接并初始化
if ur_robot.connect():
    print("✅ UR机械臂连接成功")
    
    # 获取末端位姿
    position, orientation = ur_robot.get_ee_pose()
    print(f"末端位置: {position}")
    print(f"末端姿态: {orientation}")
```

### MuJoCoRobotInterface (仿真接口)

MuJoCo仿真环境中的机器人接口。

```python
from src.robot_models.mujoco_robot_interface import MuJoCoRobotInterface

# 创建仿真机器人
sim_robot = MuJoCoRobotInterface(
    robot_type=RobotType.UNITREE_D1,
    xml_path="assets/aloha/aloha.xml",
    headless=False,
    render_mode="human"
)

# 运行仿真
sim_robot.reset()
for _ in range(1000):
    sim_robot.step()
    time.sleep(0.01)
```

## 🎮 控制算法

### MasterSlaveController (主从控制器)

主从控制的核心算法实现。

```python
from src.control.master_slave_control import MasterSlaveController

# 创建主从控制器
controller = MasterSlaveController(
    master_robot=master_robot,
    slave_robot=slave_robot,
    control_frequency=50.0
)

# 设置主从变换
transform_matrix = np.eye(4)
transform_matrix[:3, 3] = np.array([0.1, 0.0, 0.0])  # 偏移0.1米
controller.set_master_to_slave_transform(transform_matrix)

# 启动控制循环
controller.start_control()

# 运行指定时长
controller.run_control_loop(duration=60.0)

# 停止控制
controller.stop_control()
```

### RealMasterSlaveControl (真实机器人控制)

针对真实机器人的主从控制实现。

```python
from src.control.real_master_slave_control import RealMasterSlaveControl

# 创建真实机器人控制器
real_controller = RealMasterSlaveControl(
    master_robot=master_robot,
    slave_robot=slave_robot,
    safety_manager=safety_manager
)

# 配置安全参数
real_controller.set_safety_limits(
    max_joint_velocity=0.5,
    max_joint_acceleration=2.0,
    max_ee_velocity=0.3
)

# 启动实时控制
real_controller.start_realtime_control()
```

### GymMasterSlaveControl (强化学习控制)

基于强化学习的主从控制算法。

```python
from src.control.gym_master_slave_control import GymMasterSlaveControl

# 创建强化学习控制器
gym_controller = GymMasterSlaveControl(
    master_robot=master_robot,
    slave_robot=slave_robot,
    env_config=env_config
)

# 训练模型
gym_controller.train(
    total_timesteps=1000000,
    save_path="./models/master_slave_policy"
)

# 加载训练好的模型
gym_controller.load_model("./models/master_slave_policy")
```

## 🔬 运动学计算

### ForwardKinematics (正向运动学)

计算关节角度到末端位姿的映射。

```python
from src.kinematics.forward_kinematics import ForwardKinematics

# 创建正向运动学求解器
fk_solver = ForwardKinematics(robot_type=RobotType.UNITREE_D1)

# 计算末端位姿
joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
position, orientation = fk_solver.forward_kinematics(joint_angles)

print(f"末端位置: {position}")
print(f"末端姿态: {orientation}")
```

### InverseKinematics (逆运动学)

计算末端位姿到关节角度的映射。

```python
from src.kinematics.inverse_kinematics import InverseKinematics

# 创建逆运动学求解器
ik_solver = InverseKinematics(robot_type=RobotType.UNITREE_D1)

# 目标位姿
target_position = np.array([0.5, 0.0, 0.3])
target_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # 四元数

# 求解逆运动学
joint_solutions, success = ik_solver.ik_optimization_method(
    target_position, 
    target_orientation,
    initial_guess=np.zeros(6)
)

if success:
    print(f"找到解: {joint_solutions}")
else:
    print("未找到可行解")
```

### PinocchioIK (高级逆运动学)

基于Pinocchio的高级逆运动学求解器。

```python
from src.kinematics.pinocchio_ik import PinocchioIK

# 创建Pinocchio IK求解器
pinocchio_ik = PinocchioIK(
    urdf_path="assets/d1_550/urdf/d1_550.urdf",
    mesh_path="assets/d1_550/meshes/"
)

# 设置求解参数
pinocchio_ik.set_solver_parameters(
    max_iterations=100,
    tolerance=1e-6,
    damping_factor=1e-3
)

# 求解逆运动学
joint_solutions = pinocchio_ik.solve_ik(
    target_position,
    target_orientation,
    initial_guess
)
```

## 🛡️ 安全约束

### SafetyConstraintManager (安全约束管理器)

管理机器人的安全约束和限制。

```python
from src.utils.safety_constraints import SafetyConstraintManager

# 创建安全约束管理器
safety_manager = SafetyConstraintManager()

# 设置关节限制
safety_manager.set_joint_limits(
    lower_limits=np.array([-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]),
    upper_limits=np.array([3.14, 3.14, 3.14, 3.14, 3.14, 3.14])
)

# 设置速度限制
safety_manager.set_velocity_limits(max_velocity=0.5)

# 检查关节状态是否安全
is_safe = safety_manager.check_joint_safety(joint_positions, joint_velocities)

if not is_safe:
    print("⚠️ 关节状态超出安全范围")
```

## 📊 工具函数

### MathUtils (数学工具)

常用的数学计算函数。

```python
from src.utils.math_utils import MathUtils

# 四元数操作
quat1 = np.array([1.0, 0.0, 0.0, 0.0])
quat2 = np.array([0.0, 0.0, 0.0, 1.0])

# 四元数乘法
quat_product = MathUtils.quaternion_multiply(quat1, quat2)

# 四元数到旋转矩阵
rotation_matrix = MathUtils.quaternion_to_rotation_matrix(quat1)

# 旋转矩阵到四元数
quaternion = MathUtils.rotation_matrix_to_quaternion(rotation_matrix)

# 欧拉角转换
euler_angles = MathUtils.quaternion_to_euler(quat1, order='xyz')
quaternion = MathUtils.euler_to_quaternion(euler_angles, order='xyz')
```

### Visualization (可视化工具)

数据可视化和调试工具。

```python
from src.utils.visualization import Visualization

# 创建可视化工具
viz = Visualization()

# 绘制关节轨迹
joint_trajectory = np.random.randn(100, 6)
viz.plot_joint_trajectory(joint_trajectory, title="关节轨迹")

# 绘制末端轨迹
ee_trajectory = np.random.randn(100, 3)
viz.plot_ee_trajectory(ee_trajectory, title="末端轨迹")

# 3D可视化机器人
viz.visualize_robot_3d(joint_positions, robot_type=RobotType.UNITREE_D1)

# 显示力反馈数据
force_data = np.random.randn(100, 6)
viz.plot_force_feedback(force_data, title="力反馈数据")
```

## 🔧 配置管理

### 配置文件结构

系统使用YAML格式的配置文件：

```yaml
# config/robot_config.yaml
robot:
  master:
    type: "UNITREE_D1"
    id: 1
    ip_address: "192.168.123.161"
    port: 8080
    
  slave:
    type: "UNITREE_D1"
    id: 2
    ip_address: "192.168.123.162"
    port: 8080

control:
  frequency: 50.0
  mode: "position"
  safety_enabled: true
  
kinematics:
  ik_solver: "pinocchio"
  max_iterations: 100
  tolerance: 1e-6
```

### 配置加载

```python
import yaml
from pathlib import Path

# 加载配置文件
config_path = Path("config/robot_config.yaml")
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# 使用配置
master_config = config['robot']['master']
slave_config = config['robot']['slave']
control_config = config['control']
```

## 📈 性能监控

### 性能指标

系统提供以下性能监控指标：

- **控制频率**: 实际控制循环频率
- **延迟**: 从传感器到执行器的延迟
- **精度**: 位置和姿态跟踪精度
- **稳定性**: 系统稳定性指标

### 监控接口

```python
# 获取性能指标
performance_metrics = controller.get_performance_metrics()

print(f"控制频率: {performance_metrics['control_frequency']:.2f} Hz")
print(f"平均延迟: {performance_metrics['average_latency']:.3f} ms")
print(f"位置精度: {performance_metrics['position_accuracy']:.3f} mm")
print(f"姿态精度: {performance_metrics['orientation_accuracy']:.3f} deg")
```

## 🚨 错误处理

### 异常类型

系统定义了以下异常类型：

```python
class RobotConnectionError(Exception):
    """机器人连接错误"""
    pass

class SafetyViolationError(Exception):
    """安全约束违反错误"""
    pass

class KinematicsError(Exception):
    """运动学计算错误"""
    pass

class ControlError(Exception):
    """控制算法错误"""
    pass
```

### 错误处理示例

```python
try:
    # 连接机器人
    robot.connect()
    
    # 执行控制
    controller.start_control()
    
except RobotConnectionError as e:
    print(f"机器人连接失败: {e}")
    # 重试连接
    
except SafetyViolationError as e:
    print(f"安全约束违反: {e}")
    # 停止运动，进入安全模式
    
except KinematicsError as e:
    print(f"运动学计算错误: {e}")
    # 使用备用求解器
    
except ControlError as e:
    print(f"控制错误: {e}")
    # 切换到安全控制模式
```

## 📚 使用示例

### 完整的主从控制示例

```python
#!/usr/bin/env python3
"""
完整的主从控制示例
"""

import numpy as np
import time
from src.robot_models.d1arm_robot import D1ArmRobot
from src.control.master_slave_control import MasterSlaveController
from src.utils.safety_constraints import SafetyConstraintManager

def main():
    # 1. 创建机器人实例
    master_robot = D1ArmRobot(
        robot_id="d1_master",
        ip_address="192.168.123.161"
    )
    
    slave_robot = D1ArmRobot(
        robot_id="d1_slave", 
        ip_address="192.168.123.162"
    )
    
    # 2. 连接机器人
    if not master_robot.connect():
        print("❌ 主机械臂连接失败")
        return
        
    if not slave_robot.connect():
        print("❌ 从机械臂连接失败")
        return
    
    # 3. 创建安全约束管理器
    safety_manager = SafetyConstraintManager()
    safety_manager.set_velocity_limits(max_velocity=0.5)
    
    # 4. 创建主从控制器
    controller = MasterSlaveController(
        master_robot=master_robot,
        slave_robot=slave_robot,
        safety_manager=safety_manager,
        control_frequency=50.0
    )
    
    # 5. 设置主从变换
    transform = np.eye(4)
    transform[:3, 3] = np.array([0.1, 0.0, 0.0])  # 偏移0.1米
    controller.set_master_to_slave_transform(transform)
    
    # 6. 启动控制
    try:
        controller.start_control()
        controller.run_control_loop(duration=60.0)
    except KeyboardInterrupt:
        print("\n用户中断，正在停止...")
    finally:
        controller.stop_control()
        master_robot.disconnect()
        slave_robot.disconnect()

if __name__ == "__main__":
    main()
```

---

**📝 注意事项**:
- 所有角度单位为弧度
- 所有位置单位为米
- 时间单位为秒
- 确保在安全环境下使用
- 定期检查系统状态和性能指标 