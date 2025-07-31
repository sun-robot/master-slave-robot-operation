# 配置指南

## 📋 概述

本文档详细介绍了力反馈主从控制系统的配置选项，包括机器人参数、控制参数、安全设置等。

## ⚙️ 配置文件结构

系统使用YAML格式的配置文件，主要配置文件包括：

```
config/
├── robot_config.yaml          # 机器人基本配置
├── real_d1arm_config.yaml    # D1机械臂详细配置
├── safety_constraints.yaml    # 安全约束配置
└── ur_robot_config.yaml      # UR机械臂配置
```

## 🤖 机器人配置

### 基本机器人配置 (robot_config.yaml)

```yaml
# 机器人基本配置
robot:
  # 主机械臂配置
  master:
    type: "UNITREE_D1"           # 机器人类型
    id: 1                        # 机器人ID
    ip_address: "192.168.123.161" # IP地址
    port: 8080                   # 通信端口
    enabled: true                # 是否启用
    
  # 从机械臂配置
  slave:
    type: "UNITREE_D1"
    id: 2
    ip_address: "192.168.123.162"
    port: 8080
    enabled: true
    
  # 通用参数
  common:
    timeout: 5.0                 # 连接超时时间(秒)
    retry_attempts: 3            # 重试次数
    heartbeat_interval: 1.0      # 心跳间隔(秒)
```

### D1机械臂详细配置 (real_d1arm_config.yaml)

```yaml
# D1机械臂详细配置
d1_arm:
  # 关节限制
  joint_limits:
    joint1: [-3.14, 3.14]       # 关节1限制(弧度)
    joint2: [-3.14, 3.14]       # 关节2限制
    joint3: [-3.14, 3.14]       # 关节3限制
    joint4: [-3.14, 3.14]       # 关节4限制
    joint5: [-3.14, 3.14]       # 关节5限制
    joint6: [-3.14, 3.14]       # 关节6限制
    
  # 速度限制
  velocity_limits:
    joint1: 2.0                  # 关节1最大速度(rad/s)
    joint2: 2.0
    joint3: 2.0
    joint4: 2.0
    joint5: 2.0
    joint6: 2.0
    
  # 加速度限制
  acceleration_limits:
    joint1: 5.0                  # 关节1最大加速度(rad/s²)
    joint2: 5.0
    joint3: 5.0
    joint4: 5.0
    joint5: 5.0
    joint6: 5.0
    
  # 动力学参数
  dynamics:
    mass: [2.5, 2.0, 1.8, 1.5, 1.2, 0.8]  # 各连杆质量(kg)
    inertia: [0.1, 0.08, 0.06, 0.04, 0.03, 0.02]  # 转动惯量
    
  # 传感器配置
  sensors:
    joint_encoders: true          # 关节编码器
    force_torque_sensor: true    # 力扭矩传感器
    imu: true                    # 惯性测量单元
    
  # 通信配置
  communication:
    protocol: "UDP"              # 通信协议
    buffer_size: 1024            # 缓冲区大小
    priority: "high"             # 优先级
```

### UR机械臂配置 (ur_robot_config.yaml)

```yaml
# UR机械臂配置
ur_robot:
  # 机器人类型
  robot_type: "UR3E"             # UR3E, UR5E, UR10E
  
  # 网络配置
  network:
    robot_ip: "192.168.1.100"    # 机器人IP
    robot_port: 30002            # 机器人端口
    dashboard_port: 29999        # 仪表板端口
    
  # 关节限制(度)
  joint_limits:
    joint1: [-360, 360]
    joint2: [-360, 360]
    joint3: [-360, 360]
    joint4: [-360, 360]
    joint5: [-360, 360]
    joint6: [-360, 360]
    
  # 速度限制(度/秒)
  velocity_limits:
    joint1: 180
    joint2: 180
    joint3: 180
    joint4: 180
    joint5: 180
    joint6: 180
    
  # 安全配置
  safety:
    max_tcp_velocity: 0.25       # 最大TCP速度(m/s)
    max_tcp_acceleration: 1.2    # 最大TCP加速度(m/s²)
    max_force: 150.0             # 最大力(N)
    
  # 工具配置
  tool:
    mass: 0.5                    # 工具质量(kg)
    center_of_mass: [0.0, 0.0, 0.05]  # 重心位置
```

## 🎮 控制配置

### 主从控制参数

```yaml
# 主从控制配置
control:
  # 基本参数
  frequency: 50.0                # 控制频率(Hz)
  mode: "position"               # 控制模式: position, velocity, force
  blocking: false                # 是否阻塞执行
  
  # 主从变换
  master_slave_transform:
    translation: [0.1, 0.0, 0.0] # 平移变换(m)
    rotation: [0.0, 0.0, 0.0]   # 旋转变换(弧度)
    scaling: [1.0, 1.0, 1.0]    # 缩放因子
    
  # 控制增益
  gains:
    position_kp: [100.0, 100.0, 100.0, 50.0, 50.0, 50.0]  # 位置比例增益
    position_ki: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]        # 位置积分增益
    position_kd: [20.0, 20.0, 20.0, 10.0, 10.0, 10.0]     # 位置微分增益
    
    velocity_kp: [50.0, 50.0, 50.0, 25.0, 25.0, 25.0]     # 速度比例增益
    velocity_ki: [5.0, 5.0, 5.0, 2.5, 2.5, 2.5]           # 速度积分增益
    
  # 滤波器参数
  filters:
    joint_low_pass_cutoff: 10.0  # 关节低通滤波器截止频率(Hz)
    velocity_low_pass_cutoff: 5.0 # 速度低通滤波器截止频率(Hz)
    force_low_pass_cutoff: 20.0  # 力低通滤波器截止频率(Hz)
    
  # 轨迹规划
  trajectory:
    max_velocity: 0.3            # 最大末端速度(m/s)
    max_acceleration: 1.0        # 最大末端加速度(m/s²)
    jerk_limit: 2.0              # 最大加加速度(m/s³)
    planning_time: 0.1           # 规划时间(秒)
```

### 力反馈控制配置

```yaml
# 力反馈控制配置
force_feedback:
  # 力传感器配置
  force_sensor:
    enabled: true                # 启用力传感器
    calibration_file: "force_sensor_calibration.yaml"  # 校准文件
    noise_threshold: 0.1         # 噪声阈值(N)
    
  # 阻抗控制参数
  impedance_control:
    mass: [1.0, 1.0, 1.0]       # 虚拟质量(kg)
    damping: [10.0, 10.0, 10.0] # 虚拟阻尼(Ns/m)
    stiffness: [100.0, 100.0, 100.0]  # 虚拟刚度(N/m)
    
  # 力控制参数
  force_control:
    force_gain: [1.0, 1.0, 1.0] # 力控制增益
    max_force: 50.0              # 最大力限制(N)
    force_deadband: 0.5          # 力死区(N)
    
  # 安全参数
  safety:
    max_contact_force: 30.0      # 最大接触力(N)
    emergency_stop_force: 50.0   # 急停触发力(N)
    force_monitoring_enabled: true  # 启用力监控
```

## 🛡️ 安全配置

### 安全约束配置 (safety_constraints.yaml)

```yaml
# 安全约束配置
safety:
  # 关节限制
  joint_limits:
    enabled: true                # 启用关节限制
    soft_limits: true            # 使用软限制
    hard_limits: true            # 使用硬限制
    
  # 速度限制
  velocity_limits:
    max_joint_velocity: 0.5      # 最大关节速度(rad/s)
    max_ee_velocity: 0.3         # 最大末端速度(m/s)
    max_angular_velocity: 1.0    # 最大角速度(rad/s)
    
  # 加速度限制
  acceleration_limits:
    max_joint_acceleration: 2.0  # 最大关节加速度(rad/s²)
    max_ee_acceleration: 1.0     # 最大末端加速度(m/s²)
    
  # 力限制
  force_limits:
    max_force: 50.0              # 最大力(N)
    max_torque: 10.0             # 最大扭矩(Nm)
    
  # 碰撞检测
  collision_detection:
    enabled: true                # 启用碰撞检测
    safety_margin: 0.05          # 安全边距(m)
    max_penetration: 0.01        # 最大穿透深度(m)
    
  # 工作空间限制
  workspace_limits:
    enabled: true                # 启用工作空间限制
    x_range: [-0.8, 0.8]        # X轴范围(m)
    y_range: [-0.8, 0.8]        # Y轴范围(m)
    z_range: [0.0, 1.2]         # Z轴范围(m)
    
  # 急停配置
  emergency_stop:
    enabled: true                # 启用急停功能
    timeout: 0.1                 # 急停超时时间(秒)
    auto_recovery: false         # 自动恢复
    
  # 监控配置
  monitoring:
    enabled: true                # 启用监控
    update_rate: 100.0           # 监控更新频率(Hz)
    log_violations: true         # 记录违规事件
```

## 🔬 运动学配置

### 运动学求解器配置

```yaml
# 运动学配置
kinematics:
  # 正向运动学
  forward_kinematics:
    solver: "analytical"         # 求解器类型: analytical, numerical
    precision: 1e-6              # 计算精度
    
  # 逆运动学
  inverse_kinematics:
    solver: "pinocchio"          # 求解器类型: pinocchio, scipy, custom
    max_iterations: 100          # 最大迭代次数
    tolerance: 1e-6              # 收敛容差
    damping_factor: 1e-3         # 阻尼因子
    
  # Pinocchio特定配置
  pinocchio:
    urdf_path: "assets/d1_550/urdf/d1_550.urdf"  # URDF文件路径
    mesh_path: "assets/d1_550/meshes/"            # 网格文件路径
    collision_checking: true      # 启用碰撞检查
    visual_checking: true         # 启用可视化检查
    
  # 数值求解器配置
  numerical_solver:
    method: "BFGS"               # 优化方法: BFGS, L-BFGS-B, SLSQP
    bounds: true                 # 使用边界约束
    constraints: true             # 使用约束条件
```

## 📊 数据记录配置

### 数据记录参数

```yaml
# 数据记录配置
data_logging:
  # 基本设置
  enabled: true                  # 启用数据记录
  log_directory: "./logs"        # 日志目录
  log_format: "csv"              # 日志格式: csv, hdf5, json
  
  # 记录内容
  record_joint_states: true      # 记录关节状态
  record_ee_poses: true          # 记录末端位姿
  record_forces: true            # 记录力数据
  record_commands: true          # 记录控制命令
  record_errors: true            # 记录错误信息
  
  # 记录频率
  joint_states_rate: 50.0        # 关节状态记录频率(Hz)
  ee_poses_rate: 50.0           # 末端位姿记录频率(Hz)
  forces_rate: 100.0            # 力数据记录频率(Hz)
  
  # 数据压缩
  compression:
    enabled: true                # 启用压缩
    algorithm: "gzip"            # 压缩算法
    level: 6                     # 压缩级别
    
  # 数据保留
  retention:
    max_file_size: "100MB"       # 最大文件大小
    max_days: 30                 # 保留天数
    auto_cleanup: true           # 自动清理
```

## 🌐 网络配置

### 网络参数设置

```yaml
# 网络配置
network:
  # 主机械臂网络
  master:
    interface: "eth0"            # 网络接口
    ip_address: "192.168.123.162"  # IP地址
    subnet_mask: "255.255.255.0"   # 子网掩码
    gateway: "192.168.123.1"    # 网关
    dns: ["8.8.8.8", "8.8.4.4"] # DNS服务器
    
  # 从机械臂网络
  slave:
    interface: "eth1"            # 网络接口
    ip_address: "192.168.124.162"  # IP地址
    subnet_mask: "255.255.255.0"   # 子网掩码
    gateway: "192.168.124.1"    # 网关
    
  # 通信参数
  communication:
    protocol: "UDP"              # 通信协议
    buffer_size: 1024            # 缓冲区大小
    timeout: 1.0                 # 超时时间(秒)
    retry_attempts: 3            # 重试次数
    
  # 防火墙配置
  firewall:
    enabled: true                # 启用防火墙
    allowed_ports: [8080, 30002] # 允许的端口
    allowed_ips: ["192.168.123.0/24"]  # 允许的IP范围
```

## 🔧 配置加载和验证

### 配置加载示例

```python
import yaml
from pathlib import Path

def load_config(config_path: str) -> dict:
    """加载配置文件"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config

def validate_config(config: dict) -> bool:
    """验证配置有效性"""
    required_keys = ['robot', 'control', 'safety']
    
    for key in required_keys:
        if key not in config:
            print(f"❌ 缺少必需的配置项: {key}")
            return False
    
    # 验证机器人配置
    robot_config = config['robot']
    if 'master' not in robot_config or 'slave' not in robot_config:
        print("❌ 缺少主从机械臂配置")
        return False
    
    # 验证控制配置
    control_config = config['control']
    if control_config['frequency'] <= 0:
        print("❌ 控制频率必须大于0")
        return False
    
    print("✅ 配置验证通过")
    return True

# 使用示例
config = load_config("config/robot_config.yaml")
if validate_config(config):
    print("配置加载成功")
else:
    print("配置验证失败")
```

### 动态配置更新

```python
def update_config(config: dict, updates: dict) -> dict:
    """动态更新配置"""
    def deep_update(d, u):
        for k, v in u.items():
            if isinstance(v, dict):
                d[k] = deep_update(d.get(k, {}), v)
            else:
                d[k] = v
        return d
    
    return deep_update(config, updates)

# 使用示例
updates = {
    'control': {
        'frequency': 100.0,  # 提高控制频率
        'gains': {
            'position_kp': [150.0, 150.0, 150.0, 75.0, 75.0, 75.0]
        }
    }
}

config = update_config(config, updates)
```

## 📝 配置最佳实践

### 1. 性能优化配置

```yaml
# 高性能配置
performance:
  control_frequency: 100.0       # 高控制频率
  real_time_priority: true       # 实时优先级
  cpu_affinity: [0, 1, 2, 3]    # CPU亲和性
  memory_pool_size: "1GB"        # 内存池大小
  
  # 网络优化
  network:
    tcp_nodelay: true            # 禁用Nagle算法
    socket_buffer_size: 65536    # 大缓冲区
    priority: "high"             # 高优先级
```

### 2. 安全配置

```yaml
# 高安全配置
safety:
  # 严格的速度限制
  velocity_limits:
    max_joint_velocity: 0.3      # 降低速度限制
    max_ee_velocity: 0.2
    
  # 增强的碰撞检测
  collision_detection:
    enabled: true
    safety_margin: 0.1           # 增大安全边距
    continuous_checking: true     # 连续检查
    
  # 多重安全机制
  emergency_stop:
    enabled: true
    hardware_stop: true          # 硬件急停
    software_stop: true          # 软件急停
    timeout: 0.05               # 更短的超时时间
```

### 3. 调试配置

```yaml
# 调试配置
debug:
  enabled: true                  # 启用调试模式
  log_level: "DEBUG"             # 详细日志
  verbose_output: true           # 详细输出
  
  # 数据记录
  data_logging:
    enabled: true
    record_all: true             # 记录所有数据
    high_frequency: true         # 高频记录
    
  # 可视化
  visualization:
    enabled: true                # 启用可视化
    real_time_plotting: true     # 实时绘图
    save_plots: true             # 保存图表
```

## 🚨 常见配置问题

### 1. 网络连接问题

```bash
# 检查网络配置
ip addr show eth0

# 测试连接
ping 192.168.123.161

# 检查防火墙
sudo ufw status
sudo ufw allow 8080
```

### 2. 性能问题

```bash
# 检查系统资源
htop
free -h
iostat

# 优化系统参数
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3. 安全约束问题

```python
# 检查安全约束
safety_manager = SafetyConstraintManager()
is_safe = safety_manager.check_all_constraints(joint_positions, joint_velocities)

if not is_safe:
    print("⚠️ 安全约束违反")
    # 记录详细信息
    safety_manager.log_violation(joint_positions, joint_velocities)
```

---

**📝 配置建议**:
- 根据实际硬件调整参数
- 逐步优化配置参数
- 定期备份配置文件
- 记录配置更改历史
- 测试配置变更的影响 