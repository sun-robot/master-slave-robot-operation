# 通用跨机械臂主从控制系统使用说明

## 概述

本系统是一个通用的跨机械臂主从控制系统，支持多种机械臂品牌和型号之间的主从协调控制。系统提供了统一的接口，使得不同品牌的机械臂可以无缝协作。

## 支持的机械臂

### 当前支持的机械臂类型
- **Unitree D1**: 7自由度机械臂
- **Universal Robots UR3e**: 6自由度协作机械臂
- **Universal Robots UR5**: 6自由度协作机械臂
- **Universal Robots UR10**: 6自由度协作机械臂
- **Franka Panda**: 7自由度协作机械臂

### 扩展支持
系统采用模块化设计，可以轻松添加新的机械臂类型。只需实现相应的接口即可。

## 快速开始

### 1. 环境准备

确保您的系统满足以下要求：
- Python 3.8+
- Ubuntu 20.04+
- 必要的机械臂驱动和SDK

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

### 3. 基本使用

#### 使用默认配置
```bash
python main.py
```

#### 指定机械臂类型
```bash
python main.py --master-type unitree_d1 --slave-type ur3e
```

#### 仿真模式
```bash
python main.py --master-mode simulation --slave-mode simulation
```

#### 硬件模式
```bash
python main.py --master-mode simulation --slave-mode hardware
```

## 详细使用说明

### 命令行参数

#### 基本参数
- `--config`: 配置文件路径 (默认: config/universal_robot_config.yaml)
- `--master-type`: 主机械臂类型 (默认: unitree_d1)
- `--slave-type`: 从机械臂类型 (默认: ur3e)
- `--master-mode`: 主机械臂模式 (simulation/hardware, 默认: simulation)
- `--slave-mode`: 从机械臂模式 (simulation/hardware, 默认: hardware)

#### 控制参数
- `--duration`: 运行时长(秒) (默认: 10.0)
- `--transform`: 空间变换参数 [x, y, z, rx, ry, rz] (默认: [0.1, 0.1, 0.0, 0.0, 0.0, 0.0])
- `--control-mode`: 控制模式 (position/velocity/force, 默认: position)

#### 调试参数
- `--verbose, -v`: 详细输出
- `--debug`: 调试模式
- `--dry-run`: 试运行模式，不实际控制机械臂

### 使用示例

#### 示例1: Unitree D1 + UR3e 主从控制
```bash
python main.py --master-type unitree_d1 --slave-type ur3e --master-mode simulation --slave-mode hardware
```

#### 示例2: 全仿真模式
```bash
python main.py --master-type ur5 --slave-type franka_panda --master-mode simulation --slave-mode simulation
```

#### 示例3: 自定义变换参数
```bash
python main.py --master-type unitree_d1 --slave-type ur3e --transform 0.2 0.1 0.0 0.0 0.0 0.0
```

#### 示例4: 调试模式
```bash
python main.py --master-type unitree_d1 --slave-type ur3e --debug --verbose
```

#### 示例5: 试运行模式
```bash
python main.py --master-type unitree_d1 --slave-type ur3e --dry-run
```

## 配置文件

系统使用YAML格式的配置文件来管理各种参数。主要配置文件包括：

### 通用配置文件 (config/universal_robot_config.yaml)
包含所有机械臂的配置信息，包括：
- 机械臂类型和模式配置
- 网络和通信参数
- 安全约束设置
- 性能优化参数

### 使用自定义配置文件
```bash
python main.py --config my_custom_config.yaml
```

## 控制模式

### 位置控制 (position)
- 直接控制机械臂末端执行器的位置和姿态
- 适用于精确的位置跟踪任务
- 参数可调：速度比例、加速度比例

### 速度控制 (velocity)
- 控制机械臂末端执行器的线速度和角速度
- 适用于连续运动任务
- 参数可调：增益、最大速度

### 力控制 (force)
- 控制机械臂末端执行器的力和力矩
- 适用于接触任务和力反馈
- 参数可调：最大力、最大力矩

## 安全特性

### 内置安全机制
1. **关节限位检查**: 确保关节角度在安全范围内
2. **工作空间限制**: 防止机械臂超出工作空间
3. **速度限制**: 限制关节和笛卡尔空间的最大速度
4. **加速度限制**: 防止过大的加速度
5. **碰撞检测**: 实时检测潜在的碰撞风险

### 紧急停止
- 支持紧急停止功能
- 可通过配置文件启用/禁用
- 提供多种触发方式

## 网络配置

### ROS集成
- 支持ROS1和ROS2
- 可配置节点名称和命名空间
- 支持话题和服务通信

### TCP/IP通信
- 支持TCP/IP直接通信
- 可配置连接超时和读写超时
- 支持实时通信优化

## 日志和调试

### 日志系统
- 支持多级别日志记录
- 可配置日志文件路径和格式
- 支持日志轮转

### 调试功能
- 详细的状态信息输出
- 轨迹记录和回放
- 可视化调试工具

## 性能优化

### 实时性能
- 控制频率可配置 (默认50Hz)
- 支持实时线程优先级设置
- 内存使用优化

### 多线程支持
- 控制线程和I/O线程分离
- 可配置线程优先级
- 支持多核CPU利用

## 故障排除

### 常见问题

#### 1. 机械臂连接失败
- 检查网络连接
- 验证IP地址和端口
- 确认机械臂电源和状态

#### 2. 控制精度问题
- 调整控制参数
- 检查机械臂校准状态
- 验证传感器数据

#### 3. 性能问题
- 检查系统资源使用
- 调整控制频率
- 优化网络配置

### 调试技巧
1. 使用 `--verbose` 参数获取详细输出
2. 使用 `--debug` 参数启用调试模式
3. 使用 `--dry-run` 参数进行配置验证
4. 查看日志文件获取错误信息

## 扩展开发

### 添加新的机械臂类型
1. 实现 `RobotInterface` 接口
2. 在 `RobotFactory` 中注册新类型
3. 更新配置文件
4. 添加相应的URDF或模型文件

### 自定义控制算法
1. 继承 `UniversalMasterSlaveController` 类
2. 重写控制方法
3. 实现自定义的安全约束

## 技术支持

如果您在使用过程中遇到问题：
1. 查看日志文件获取详细错误信息
2. 参考故障排除指南
3. 提交Issue并提供详细信息
4. 联系技术支持团队

## 版本信息

- **当前版本**: v1.0.0
- **最后更新**: 2024年12月
- **维护团队**: sunjian15@foxmail.com 