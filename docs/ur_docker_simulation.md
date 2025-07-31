# UR Docker仿真环境部署指南

## 📋 概述

本文档介绍如何使用Docker部署Universal Robots仿真环境，用于力反馈主从控制系统的开发和测试。URSim是Universal Robots的离线仿真器，可以模拟真实机器人的行为。

## 🐳 Docker环境准备

### 1. 安装Docker

```bash
# 更新包管理器
sudo apt update

# 安装Docker依赖
sudo apt install -y apt-transport-https ca-certificates curl gnupg lsb-release

# 添加Docker官方GPG密钥
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# 添加Docker仓库
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 安装Docker
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io

# 将当前用户添加到docker组
sudo usermod -aG docker $USER

# 启动Docker服务
sudo systemctl start docker
sudo systemctl enable docker

# 验证安装
docker --version
```

### 2. 配置Docker权限

```bash
# 重新登录以应用组权限
newgrp docker

# 测试Docker权限
docker run hello-world
```

## 🤖 URSim Docker部署

### 1. 基本URSim容器启动

```bash
# 启动UR5e仿真器
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  --name ursim \
  universalrobots/ursim_e-series
```

**参数说明**：
- `--rm`: 容器停止后自动删除
- `-it`: 交互模式
- `-p 5900:5900`: VNC端口映射
- `-p 6080:6080`: Web VNC端口映射
- `--name ursim`: 容器名称

### 2. 访问仿真器界面

启动容器后，可以通过以下方式访问：

- **Web VNC**: 打开浏览器访问 `http://localhost:6080/vnc.html`
- **VNC客户端**: 连接到 `localhost:5900`

### 3. 网络配置

#### 创建专用网络

```bash
# 创建Docker网络
docker network create --subnet=192.168.56.0/24 ursim_net

# 使用固定IP启动容器
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  --net ursim_net \
  --ip 192.168.56.101 \
  --name ursim \
  universalrobots/ursim_e-series
```

#### 网络访问

使用固定IP后，可以通过以下地址访问：
- **Web VNC**: `http://192.168.56.101:6080/vnc.html`
- **机器人IP**: `192.168.56.101`

## 🔧 外部控制配置

### 1. 准备外部控制环境

```bash
# 创建本地存储目录
mkdir -p ${HOME}/.ursim/programs
mkdir -p ${HOME}/.ursim/urcaps

# 下载外部控制URCap
URCAP_VERSION=1.0.5
curl -L -o ${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar
```

### 2. 启动带外部控制的容器

```bash
# 启动带外部控制的URSim容器
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -v ${HOME}/.ursim/urcaps:/urcaps \
  -v ${HOME}/.ursim/programs:/ursim/programs \
  --net ursim_net \
  --ip 192.168.56.101 \
  --name ursim \
  universalrobots/ursim_e-series
```

### 3. 配置外部控制程序

在URSim界面中：

1. **安装URCap**：
   - 进入 `设置` → `系统` → `URCap`
   - 安装 `externalcontrol-${URCAP_VERSION}.jar`

2. **创建程序**：
   - 创建新程序
   - 添加 `ExternalControl` 节点
   - 保存程序到 `/ursim/programs/` 目录

## 🚀 自动化启动脚本

### 1. 创建启动脚本

```bash
#!/bin/bash
# start_ursim.sh - URSim Docker启动脚本

set -e

# 配置参数
URCAP_VERSION="1.0.5"
NETWORK_NAME="ursim_net"
CONTAINER_NAME="ursim"
ROBOT_IP="192.168.56.101"
NETWORK_SUBNET="192.168.56.0/24"

echo "=== 启动URSim Docker仿真环境 ==="

# 检查Docker是否运行
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker未运行，请启动Docker服务"
    exit 1
fi

# 创建网络（如果不存在）
if ! docker network ls | grep -q $NETWORK_NAME; then
    echo "创建Docker网络: $NETWORK_NAME"
    docker network create --subnet=$NETWORK_SUBNET $NETWORK_NAME
fi

# 创建本地目录
echo "创建本地存储目录..."
mkdir -p ${HOME}/.ursim/programs
mkdir -p ${HOME}/.ursim/urcaps

# 下载外部控制URCap（如果不存在）
URCAP_FILE="${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar"
if [ ! -f "$URCAP_FILE" ]; then
    echo "下载外部控制URCap..."
    curl -L -o $URCAP_FILE \
        https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar
fi

# 停止现有容器（如果存在）
if docker ps -a | grep -q $CONTAINER_NAME; then
    echo "停止现有容器..."
    docker stop $CONTAINER_NAME > /dev/null 2>&1 || true
    docker rm $CONTAINER_NAME > /dev/null 2>&1 || true
fi

# 启动URSim容器
echo "启动URSim容器..."
docker run --rm -it \
    -p 5900:5900 \
    -p 6080:6080 \
    -v ${HOME}/.ursim/urcaps:/urcaps \
    -v ${HOME}/.ursim/programs:/ursim/programs \
    --net $NETWORK_NAME \
    --ip $ROBOT_IP \
    --name $CONTAINER_NAME \
    universalrobots/ursim_e-series

echo "✅ URSim仿真环境启动完成"
echo "🌐 Web VNC访问地址: http://$ROBOT_IP:6080/vnc.html"
echo "🤖 机器人IP地址: $ROBOT_IP"
```

### 2. 设置脚本权限

```bash
# 创建脚本文件
cat > start_ursim.sh << 'EOF'
# 上述脚本内容
EOF

# 设置执行权限
chmod +x start_ursim.sh

# 运行脚本
./start_ursim.sh
```

## 🔗 ROS2集成

### 1. 安装UR ROS2驱动

```bash
# 安装ROS2 UR驱动
sudo apt-get install ros-${ROS_DISTRO}-ur

# 或者从源码编译
export COLCON_WS=~/workspace/ros_ur_driver
mkdir -p $COLCON_WS/src

cd $COLCON_WS
git clone -b main https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver

# 安装依赖
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y

# 编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 启动ROS2控制

```bash
# 启动UR5e控制节点
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
```

### 3. 测试连接

```bash
# 检查机器人状态
ros2 topic echo /joint_states

# 检查机器人信息
ros2 service call /get_robot_mode ur_robot_driver/srv/GetRobotMode

# 测试关节控制
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [{
    positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}"
```

## 🔧 力反馈主从控制集成

### 1. 配置UR机械臂

编辑 `config/ur_robot_config.yaml`:

```yaml
# UR机械臂仿真配置
ur_robot:
  robot_type: "UR5E"
  robot_ip: "192.168.56.101"  # Docker容器IP
  robot_port: 30002
  dashboard_port: 29999
  
  # 仿真环境参数
  simulation:
    enabled: true
    urcap_installed: true
    external_control_enabled: true
    
  # 网络配置
  network:
    interface: "docker0"
    timeout: 5.0
    retry_attempts: 3
```

### 2. 启动主从控制

```bash
# 启动UR仿真环境
./start_ursim.sh &

# 等待仿真器启动
sleep 30

# 启动主从控制
python demo_master_slave.py \
  --master-id 1 \
  --slave-id 2 \
  --duration 60 \
  --transform 0.1 0.0 0.0 0.0 0.0 0.0
```

### 3. 验证控制效果

```python
#!/usr/bin/env python3
"""
UR仿真环境测试脚本
"""

import time
import numpy as np
from src.robot_models.ur_robot import URRobot
from src.robot_models.robot_interface import RobotType

def test_ur_simulation():
    """测试UR仿真环境"""
    print("=== 测试UR仿真环境 ===")
    
    # 创建UR机器人实例
    ur_robot = URRobot(
        robot_type=RobotType.UR5E,
        robot_id="ur5e_sim",
        node_name='ur5e_sim_controller'
    )
    
    try:
        # 连接机器人
        if ur_robot.connect():
            print("✅ UR仿真器连接成功")
            
            # 获取关节状态
            joint_pos = ur_robot.get_joint_positions()
            print(f"关节位置: {joint_pos}")
            
            # 获取末端位姿
            position, orientation = ur_robot.get_ee_pose()
            print(f"末端位置: {position}")
            print(f"末端姿态: {orientation}")
            
            # 测试简单运动
            target_positions = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
            ur_robot.move_to_joint_positions(target_positions, blocking=True)
            print("✅ 运动测试完成")
            
        else:
            print("❌ UR仿真器连接失败")
            
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
    finally:
        ur_robot.disconnect()

if __name__ == "__main__":
    test_ur_simulation()
```

## 🛠️ 故障排除

### 1. Docker相关问题

```bash
# 检查Docker状态
docker ps -a

# 查看容器日志
docker logs ursim

# 重启Docker服务
sudo systemctl restart docker

# 清理Docker资源
docker system prune -a
```

### 2. 网络连接问题

```bash
# 检查网络连接
ping 192.168.56.101

# 检查端口是否开放
telnet 192.168.56.101 30002

# 检查Docker网络
docker network ls
docker network inspect ursim_net
```

### 3. ROS2连接问题

```bash
# 检查ROS2环境
echo $ROS_DISTRO
ros2 --version

# 检查UR驱动安装
ros2 pkg list | grep ur

# 测试UR驱动
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
```

### 4. 常见错误及解决方案

#### 错误：容器启动失败
```bash
# 解决方案：检查Docker权限
sudo usermod -aG docker $USER
newgrp docker
```

#### 错误：网络连接超时
```bash
# 解决方案：重新创建网络
docker network rm ursim_net
docker network create --subnet=192.168.56.0/24 ursim_net
```

#### 错误：URCap安装失败
```bash
# 解决方案：手动下载URCap
curl -L -o externalcontrol.jar \
  https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.jar
```

## 📊 性能优化

### 1. Docker性能优化

```bash
# 增加Docker内存限制
echo '{"memory": "4g", "memory-swap": "4g"}' | sudo tee /etc/docker/daemon.json
sudo systemctl restart docker

# 优化Docker存储
docker system prune -a
```

### 2. 网络性能优化

```bash
# 优化网络参数
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3. 实时性能优化

```bash
# 安装实时内核（推荐）
sudo apt install -y linux-image-rt

# 设置CPU性能模式
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## 📚 参考资源

- [Universal Robots ROS2 Driver Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)
- [URSim Docker Image](https://hub.docker.com/r/universalrobots/ursim_e-series)
- [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap)
- [ROS2 UR Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

**⚠️ 重要提醒**:
- 确保Docker服务正常运行
- 使用固定IP地址避免网络冲突
- 定期清理Docker资源
- 在安全环境下测试仿真功能 