# 力反馈主从控制系统部署指南

## 📋 系统要求

### 硬件要求
- **CPU**: Intel i7 或 AMD Ryzen 7 及以上
- **内存**: 16GB RAM 最低，32GB 推荐
- **存储**: 50GB 可用空间
- **网络**: 千兆以太网连接
- **机械臂**: 2台 Unitree D1 机械臂（或兼容型号）

### 软件要求
- **操作系统**: Ubuntu 18.04 LTS 或 Ubuntu 20.04 LTS
- **Python**: 3.8 或 3.9
- **CUDA**: 11.0+ (用于GPU加速，可选)
- **ROS**: Noetic (如果使用UR机械臂)

## 🚀 安装步骤

### 1. 系统环境准备

#### 更新系统包
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git wget curl
```

#### 安装Python依赖
```bash
# 安装Python 3.8+ (如果未安装)
sudo apt install -y python3 python3-pip python3-venv

# 创建虚拟环境
python3 -m venv force_link_env
source force_link_env/bin/activate
```

### 2. 安装核心依赖

#### 基础科学计算库
```bash
pip install numpy>=1.21.0 scipy>=1.7.0 matplotlib>=3.5.0
pip install pyyaml>=6.0 transforms3d>=0.3.1 sympy>=1.9
pip install opencv-python>=4.5.0 pandas>=1.3.0 scikit-learn>=1.0.0
```

#### 机器人控制库
```bash
# 安装MuJoCo (可选，用于仿真)
pip install mujoco>=2.3.0

# 安装Pinocchio (可选，用于高级运动学)
conda install pinocchio==3.1.0 -c conda-forge

# 安装CasADi (用于优化求解)
pip install casadi>=3.5.0 meshcat>=0.3.0
```

#### 深度学习框架 (可选)
```bash
# 安装PyTorch (用于Isaac Gym)
pip install torch>=1.10.0 tensorboard>=2.8.0
```

### 3. 安装Unitree SDK

#### 下载SDK
```bash
# 克隆Unitree SDK2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2

# 安装Python SDK
pip install -e .
```

#### 配置网络
```bash
# 设置机械臂IP地址
sudo ip addr add 192.168.123.162/24 dev eth0

# 测试连接
ping 192.168.123.161  # 机械臂默认IP
```

### 4. 安装ROS (可选)

如果使用UR机械臂作为从端：

```bash
# 安装ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# 安装UR ROS驱动
sudo apt install -y ros-noetic-ur-robot-driver
```

### 5. 安装Isaac Gym (可选)

用于高级仿真：

```bash
# 从NVIDIA官网下载Isaac Gym
# https://developer.nvidia.com/isaac-gym

# 解压并安装
cd isaacgym/python
pip install -e .
```

## ⚙️ 配置系统

### 1. 机械臂配置

#### D1机械臂配置
编辑 `config/real_d1arm_config.yaml`:

```yaml
# D1机械臂配置
d1_arm:
  master:
    robot_id: 1
    ip_address: "192.168.123.161"
    port: 8080
    joint_limits:
      - [-3.14, 3.14]  # 关节1限制
      - [-3.14, 3.14]  # 关节2限制
      - [-3.14, 3.14]  # 关节3限制
      - [-3.14, 3.14]  # 关节4限制
      - [-3.14, 3.14]  # 关节5限制
      - [-3.14, 3.14]  # 关节6限制
    
  slave:
    robot_id: 2
    ip_address: "192.168.123.162"
    port: 8080
    joint_limits:
      - [-3.14, 3.14]
      - [-3.14, 3.14]
      - [-3.14, 3.14]
      - [-3.14, 3.14]
      - [-3.14, 3.14]
      - [-3.14, 3.14]
```

#### UR机械臂配置
编辑 `config/ur_robot_config.yaml`:

```yaml
# UR机械臂配置
ur_robot:
  robot_type: "UR3E"
  robot_ip: "192.168.1.100"
  robot_port: 30002
  joint_limits:
    - [-360, 360]  # 度
    - [-360, 360]
    - [-360, 360]
    - [-360, 360]
    - [-360, 360]
    - [-360, 360]
```

### 2. 安全配置

编辑 `config/safety_constraints.yaml`:

```yaml
# 安全约束配置
safety:
  max_joint_velocity: 0.5  # rad/s
  max_joint_acceleration: 2.0  # rad/s²
  max_ee_velocity: 0.3  # m/s
  max_force: 50.0  # N
  emergency_stop_enabled: true
  collision_detection_enabled: true
```

### 3. 网络配置

#### 设置静态IP
```bash
# 编辑网络配置
sudo nano /etc/netplan/01-netcfg.yaml

# 添加以下配置
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      addresses:
        - 192.168.123.162/24
      gateway4: 192.168.123.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# 应用配置
sudo netplan apply
```

## 🔧 验证安装

### 1. 测试Python环境
```bash
python -c "
import numpy as np
import scipy as sp
import mujoco as mj
print('✅ 基础依赖安装成功')
"
```

### 2. 测试Unitree SDK
```bash
python -c "
import unitree_sdk2py as sdk
print('✅ Unitree SDK安装成功')
"
```

### 3. 测试机械臂连接
```bash
# 运行连接测试
python test_joint_offset_demo.py
```

### 4. 运行基本演示
```bash
# 运行主从控制演示
python demo_master_slave.py --duration 10
```

## 🚨 故障排除

### 常见问题

#### 1. SDK连接失败
```bash
# 检查网络连接
ping 192.168.123.161

# 检查防火墙
sudo ufw status
sudo ufw allow 8080

# 重启网络服务
sudo systemctl restart networking
```

#### 2. 机械臂无响应
```bash
# 检查机械臂电源
# 检查急停按钮状态
# 检查关节限位开关

# 重启机械臂控制器
# 重新初始化SDK连接
```

#### 3. 运动学求解失败
```bash
# 检查目标位姿是否可达
# 调整IK求解参数
# 检查关节限制配置
```

#### 4. 性能问题
```bash
# 检查CPU使用率
htop

# 检查内存使用
free -h

# 优化控制频率
# 降低可视化更新频率
```

## 📊 性能优化

### 1. 系统优化
```bash
# 设置CPU性能模式
sudo cpupower frequency-set -g performance

# 禁用不必要的服务
sudo systemctl disable bluetooth
sudo systemctl disable cups
```

### 2. 网络优化
```bash
# 优化网络参数
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3. 实时内核 (可选)
```bash
# 安装实时内核
sudo apt install -y linux-image-rt

# 配置GRUB使用实时内核
sudo update-grub
```

## 🔐 安全配置

### 1. 用户权限
```bash
# 创建专用用户
sudo adduser force_link_user
sudo usermod -aG dialout force_link_user

# 设置sudo权限
sudo visudo
# 添加: force_link_user ALL=(ALL) NOPASSWD: /usr/bin/python3
```

### 2. 防火墙配置
```bash
# 配置防火墙
sudo ufw enable
sudo ufw allow 8080/tcp  # Unitree SDK端口
sudo ufw allow 30002/tcp  # UR机械臂端口
```

### 3. 日志监控
```bash
# 创建日志目录
sudo mkdir -p /var/log/force_link
sudo chown force_link_user:force_link_user /var/log/force_link

# 配置日志轮转
sudo nano /etc/logrotate.d/force_link
```

## 📈 监控和维护

### 1. 系统监控
```bash
# 安装监控工具
sudo apt install -y htop iotop nethogs

# 创建监控脚本
cat > monitor_system.sh << 'EOF'
#!/bin/bash
echo "=== 系统状态监控 ==="
echo "CPU使用率: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}')%"
echo "内存使用: $(free -h | grep Mem | awk '{print $3"/"$2}')"
echo "网络连接: $(netstat -an | grep :8080 | wc -l) 个连接"
EOF
chmod +x monitor_system.sh
```

### 2. 定期维护
```bash
# 创建维护脚本
cat > maintenance.sh << 'EOF'
#!/bin/bash
echo "=== 系统维护 ==="
sudo apt update
sudo apt upgrade -y
pip install --upgrade pip
pip list --outdated | cut -d' ' -f1 | xargs -n1 pip install -U
EOF
chmod +x maintenance.sh
```

## 📚 参考资源

- [Unitree D1官方文档](https://www.unitree.com/)
- [Unitree SDK2 GitHub](https://github.com/unitreerobotics/unitree_sdk2)
- [MuJoCo文档](https://mujoco.readthedocs.io/)
- [ROS Noetic文档](http://wiki.ros.org/noetic)
- [Isaac Gym文档](https://developer.nvidia.com/isaac-gym)

---

**⚠️ 重要提醒**: 
- 部署前请确保工作环境安全
- 定期备份配置文件和代码
- 保持系统更新和安全补丁
- 记录所有配置更改和问题解决方案 