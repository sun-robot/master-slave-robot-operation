#!/bin/bash

# 通用跨机械臂主从控制系统安装脚本
# 适用于Ubuntu 20.04+

set -e

echo "=========================================="
echo "通用跨机械臂主从控制系统安装脚本"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查系统要求
check_system_requirements() {
    print_info "检查系统要求..."
    
    # 检查操作系统
    if [[ "$OSTYPE" != "linux-gnu"* ]]; then
        print_error "此脚本仅支持Linux系统"
        exit 1
    fi
    
    # 检查Ubuntu版本
    if command -v lsb_release >/dev/null 2>&1; then
        ubuntu_version=$(lsb_release -rs)
        if [[ "$ubuntu_version" != "20.04" && "$ubuntu_version" != "22.04" ]]; then
            print_warning "推荐使用Ubuntu 20.04或22.04，当前版本: $ubuntu_version"
        fi
    fi
    
    # 检查Python版本
    if command -v python3 >/dev/null 2>&1; then
        python_version=$(python3 --version | cut -d' ' -f2)
        if [[ "$python_version" < "3.8" ]]; then
            print_error "需要Python 3.8或更高版本，当前版本: $python_version"
            exit 1
        fi
        print_success "Python版本检查通过: $python_version"
    else
        print_error "未找到Python3，请先安装Python 3.8+"
        exit 1
    fi
    
    # 检查pip
    if ! command -v pip3 >/dev/null 2>&1; then
        print_error "未找到pip3，请先安装pip"
        exit 1
    fi
    
    print_success "系统要求检查完成"
}

# 更新系统包
update_system() {
    print_info "更新系统包..."
    sudo apt update
    sudo apt upgrade -y
    print_success "系统包更新完成"
}

# 安装系统依赖
install_system_dependencies() {
    print_info "安装系统依赖..."
    
    # 基础开发工具
    sudo apt install -y build-essential cmake pkg-config
    
    # Python开发工具
    sudo apt install -y python3-dev python3-pip python3-venv
    
    # 图形库依赖
    sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev
    
    # 数学库
    sudo apt install -y libblas-dev liblapack-dev
    
    # 其他依赖
    sudo apt install -y git wget curl unzip
    
    print_success "系统依赖安装完成"
}

# 安装ROS（可选）
install_ros() {
    print_info "安装ROS依赖..."
    
    # 检查是否已安装ROS
    if command -v roscore >/dev/null 2>&1; then
        print_success "ROS已安装"
        return
    fi
    
    # 添加ROS源
    if [[ ! -f /etc/apt/sources.list.d/ros-latest.list ]]; then
        print_info "添加ROS源..."
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        sudo apt update
    fi
    
    # 安装ROS Noetic（Ubuntu 20.04）
    if [[ "$(lsb_release -rs)" == "20.04" ]]; then
        print_info "安装ROS Noetic..."
        sudo apt install -y ros-noetic-desktop-full
        sudo apt install -y ros-noetic-ur-msgs ros-noetic-ur-robot-driver
    # 安装ROS Humble（Ubuntu 22.04）
    elif [[ "$(lsb_release -rs)" == "22.04" ]]; then
        print_info "安装ROS Humble..."
        sudo apt install -y ros-humble-desktop-full
        sudo apt install -y ros-humble-ur-msgs ros-humble-ur-robot-driver
    else
        print_warning "未检测到支持的Ubuntu版本，跳过ROS安装"
        return
    fi
    
    print_success "ROS安装完成"
}

# 创建Python虚拟环境
create_virtual_environment() {
    print_info "创建Python虚拟环境..."
    
    if [[ -d "venv" ]]; then
        print_warning "虚拟环境已存在，跳过创建"
        return
    fi
    
    python3 -m venv venv
    print_success "虚拟环境创建完成"
}

# 激活虚拟环境并安装Python依赖
install_python_dependencies() {
    print_info "安装Python依赖..."
    
    # 激活虚拟环境
    source venv/bin/activate
    
    # 升级pip
    pip install --upgrade pip
    
    # 安装基础依赖
    pip install -r requirements.txt
    
    print_success "Python依赖安装完成"
}

# 下载机械臂模型文件
download_robot_models() {
    print_info "检查机械臂模型文件..."
    
    # 检查assets目录
    if [[ ! -d "assets" ]]; then
        print_warning "assets目录不存在，请手动下载机械臂模型文件"
        return
    fi
    
    # 检查关键模型文件
    missing_models=()
    
    # Unitree D1模型
    if [[ ! -f "assets/d1_550/mujoco/d1_550.xml" ]]; then
        missing_models+=("Unitree D1")
    fi
    
    # UR模型
    for ur_type in "ur3e" "ur5" "ur10"; do
        if [[ ! -d "assets/$ur_type" ]]; then
            missing_models+=("UR $ur_type")
        fi
    done
    
    # Franka Panda模型
    if [[ ! -d "assets/franka_panda" ]]; then
        missing_models+=("Franka Panda")
    fi
    
    if [[ ${#missing_models[@]} -gt 0 ]]; then
        print_warning "缺少以下机械臂模型文件:"
        for model in "${missing_models[@]}"; do
            echo "  - $model"
        done
        print_info "请从相应厂商官网下载模型文件并放置在assets目录下"
    else
        print_success "机械臂模型文件检查完成"
    fi
}

# 创建必要的目录
create_directories() {
    print_info "创建必要的目录..."
    
    mkdir -p logs
    mkdir -p trajectories
    mkdir -p config
    mkdir -p docs
    
    print_success "目录创建完成"
}

# 设置环境变量
setup_environment() {
    print_info "设置环境变量..."
    
    # 创建环境变量文件
    cat > .env << EOF
# 通用跨机械臂主从控制系统环境变量
export PYTHONPATH=\$PYTHONPATH:$(pwd)
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
EOF
    
    print_success "环境变量设置完成"
}

# 创建启动脚本
create_launch_scripts() {
    print_info "创建启动脚本..."
    
    # 创建激活虚拟环境的脚本
    cat > activate_env.sh << 'EOF'
#!/bin/bash
# 激活虚拟环境脚本
source venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(pwd)
echo "虚拟环境已激活"
echo "使用方法: python main.py"
EOF
    
    chmod +x activate_env.sh
    
    # 创建快速启动脚本
    cat > run_demo.sh << 'EOF'
#!/bin/bash
# 快速启动演示脚本
source venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(pwd)
python main.py "$@"
EOF
    
    chmod +x run_demo.sh
    
    print_success "启动脚本创建完成"
}

# 运行测试
run_tests() {
    print_info "运行系统测试..."
    
    source venv/bin/activate
    
    # 检查Python模块导入
    if python -c "import numpy; import scipy; import yaml; print('基础依赖检查通过')" 2>/dev/null; then
        print_success "基础依赖测试通过"
    else
        print_error "基础依赖测试失败"
        exit 1
    fi
    
    # 检查配置文件
    if [[ -f "config/universal_robot_config.yaml" ]]; then
        if python -c "import yaml; yaml.safe_load(open('config/universal_robot_config.yaml'))" 2>/dev/null; then
            print_success "配置文件测试通过"
        else
            print_error "配置文件格式错误"
            exit 1
        fi
    else
        print_warning "配置文件不存在，跳过测试"
    fi
    
    print_success "系统测试完成"
}

# 显示安装完成信息
show_completion_info() {
    echo ""
    echo "=========================================="
    print_success "安装完成！"
    echo "=========================================="
    echo ""
    echo "使用说明："
    echo "1. 激活虚拟环境: source venv/bin/activate"
    echo "2. 运行演示: python main.py"
    echo "3. 查看帮助: python main.py --help"
    echo ""
    echo "或者使用快速启动脚本："
    echo "1. 激活环境: ./activate_env.sh"
    echo "2. 运行演示: ./run_demo.sh"
    echo ""
    echo "配置文件位置: config/universal_robot_config.yaml"
    echo "日志文件位置: logs/"
    echo ""
    echo "如需技术支持，请联系: sunjian15@foxmail.com"
    echo ""
}

# 主安装流程
main() {
    print_info "开始安装通用跨机械臂主从控制系统..."
    
    check_system_requirements
    update_system
    install_system_dependencies
    install_ros
    create_virtual_environment
    install_python_dependencies
    download_robot_models
    create_directories
    setup_environment
    create_launch_scripts
    run_tests
    show_completion_info
}

# 运行主函数
main "$@" 