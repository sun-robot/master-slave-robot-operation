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