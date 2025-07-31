# 力反馈主从控制系统

基于Unitree D1机械臂的主从协调控制系统，支持力反馈和实时控制。

## 🚀 快速开始

### 环境要求
- Python 3.8+
- Ubuntu 20.04+
- 1台Unitree D1机械臂 1台其它机械臂

### 安装依赖
```bash
pip install -r requirements.txt
```

### 运行演示
```bash
# 基本主从控制
python demo_master_slave.py

# 指定机械臂ID
python demo_master_slave.py --master-id 1 --slave-id 2

# 设置运行时长
python demo_master_slave.py --duration 60
```

## 📁 项目结构

```
├── src/
│   ├── control/          # 控制算法
│   ├── kinematics/       # 运动学计算
│   ├── robot_models/     # 机器人模型
│   └── utils/           # 工具函数
├── config/              # 配置文件
├── assets/              # 机器人模型资源
└── demo_master_slave.py # 主演示程序
```

## 🔧 核心功能

- **主从控制**: 实时跟随主机械臂运动
- **力反馈**: 支持力传感器数据反馈
- **安全约束**: 内置安全限制和保护机制
- **多机器人支持**: 支持多种机械臂型号

## ⚙️ 配置选项

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--master-id` | 主机械臂ID | 1 |
| `--slave-id` | 从机械臂ID | 2 |
| `--duration` | 运行时长(秒) | 无限制 |
| `--transform` | 空间变换参数 | [0,0,0,0,0,0] |

## 🔐 安全特性

- 关节变化限制
- 数据有效性检查
- 紧急停止功能
- 连接状态监控

## 📚 文档概览

本系统提供以下详细文档，帮助您快速上手和深入使用：

### 🚀 快速开始
- **[部署指南](docs/deployment.md)** - 完整的系统安装和部署流程
  - 系统要求检查
  - 依赖安装步骤
  - 网络配置说明
  - 验证安装方法
- **[UR Docker仿真环境](docs/ur_docker_simulation.md)** - UR机器人仿真环境部署
  - Docker环境配置
  - URSim仿真器部署
  - ROS2集成指南
  - 主从控制测试

### 🔧 使用指南
- **[API文档](docs/api.md)** - 完整的编程接口文档
  - 机器人模型接口
  - 控制算法API
  - 运动学计算接口
  - 安全约束管理
  - 工具函数说明

- **[配置指南](docs/configuration.md)** - 详细的配置参数说明
  - 机器人参数配置
  - 控制参数设置
  - 安全约束配置
  - 网络参数优化
  - 性能调优建议

### 🛠️ 故障排除
- **[故障排除指南](docs/troubleshooting.md)** - 常见问题和解决方案
  - 连接问题诊断
  - 控制问题解决
  - 性能优化方法
  - 安全约束处理
  - 调试工具使用

## 📖 文档使用建议

### 新手用户
1. 首先阅读 **[部署指南](docs/deployment.md)** 完成系统安装
2. 参考 **[配置指南](docs/configuration.md)** 进行基本配置
3. 遇到问题时查看 **[故障排除指南](docs/troubleshooting.md)**

### 开发者用户
1. 详细阅读 **[API文档](docs/api.md)** 了解编程接口
2. 参考 **[配置指南](docs/configuration.md)** 进行高级配置
3. 使用 **[故障排除指南](docs/troubleshooting.md)** 进行问题诊断

### 系统管理员
1. 重点关注 **[部署指南](docs/deployment.md)** 中的系统要求
2. 参考 **[配置指南](docs/configuration.md)** 进行系统优化
3. 使用 **[故障排除指南](docs/troubleshooting.md)** 进行系统维护

## 🔍 快速查找

### 按功能查找
- **安装部署**: [部署指南](docs/deployment.md)
- **仿真环境**: [UR Docker仿真环境](docs/ur_docker_simulation.md)
- **编程开发**: [API文档](docs/api.md)
- **系统配置**: [配置指南](docs/configuration.md)
- **问题解决**: [故障排除指南](docs/troubleshooting.md)

### 按问题类型查找
- **连接问题**: [故障排除指南](docs/troubleshooting.md#连接问题)
- **控制问题**: [故障排除指南](docs/troubleshooting.md#控制问题)
- **性能问题**: [故障排除指南](docs/troubleshooting.md#性能问题)
- **配置问题**: [配置指南](docs/configuration.md)

## 📝 文档更新

本文档会定期更新，最新版本请查看：
- 主项目仓库: [GitHub Repository](https://github.com/your-repo/force-link-master-slave)
- 在线文档: [Documentation Website](https://docs.your-project.com)

## 🤝 贡献指南

如果您发现文档中的错误或有改进建议，欢迎：
1. 提交 Issue 报告问题
2. 提交 Pull Request 贡献改进
3. 联系技术支持团队

## 📞 技术支持

如果您在使用过程中遇到问题：
1. 首先查看 **[故障排除指南](docs/troubleshooting.md)**
2. 搜索已有的 Issue 和解决方案
3. 提交新的 Issue 并提供详细信息
4. 联系技术支持团队获取帮助

## ⚠️ 安全提醒

使用前请确保工作空间安全，操作时保持安全距离，随时准备紧急停止。

---

**📚 文档版本**: v1.0.0  
**最后更新**: 2024年12月  
**维护团队**: sunjian15@foxmail.com 