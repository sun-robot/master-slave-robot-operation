# 故障排除指南

## 📋 概述

本文档提供了力反馈主从控制系统的常见问题和解决方案，帮助用户快速诊断和解决系统问题。

## 🔍 问题诊断流程

### 1. 系统状态检查

```bash
# 检查系统基本信息
echo "=== 系统信息 ==="
uname -a
python --version
pip list | grep -E "(numpy|scipy|mujoco|unitree)"

# 检查网络连接
echo "=== 网络状态 ==="
ip addr show
ping -c 3 192.168.123.161
ping -c 3 192.168.123.162

# 检查进程状态
echo "=== 进程状态 ==="
ps aux | grep python
ps aux | grep ros
```

### 2. 日志分析

```bash
# 查看系统日志
sudo journalctl -f

# 查看应用程序日志
tail -f /var/log/force_link/app.log

# 查看错误日志
grep -i error /var/log/force_link/*.log
```

## 🚨 常见问题及解决方案

### 1. 连接问题

#### 问题：SDK连接失败

**症状**：
```
❌ Unitree SDK连接失败
Connection timeout after 5.0 seconds
```

**可能原因**：
- 网络配置错误
- 机械臂未开机
- 防火墙阻止连接
- IP地址配置错误

**解决方案**：

```bash
# 1. 检查网络配置
ip addr show eth0

# 2. 设置正确的IP地址
sudo ip addr add 192.168.123.162/24 dev eth0

# 3. 测试网络连通性
ping -c 3 192.168.123.161

# 4. 检查防火墙
sudo ufw status
sudo ufw allow 8080/tcp

# 5. 重启网络服务
sudo systemctl restart networking
```

**验证连接**：
```python
import unitree_sdk2py as sdk

try:
    # 创建SDK实例
    robot = sdk.Robot("192.168.123.161")
    
    # 测试连接
    if robot.connect():
        print("✅ SDK连接成功")
        joint_pos = robot.get_joint_positions()
        print(f"关节位置: {joint_pos}")
    else:
        print("❌ SDK连接失败")
        
except Exception as e:
    print(f"连接异常: {e}")
```

#### 问题：ROS连接失败

**症状**：
```
❌ ROS节点连接失败
Failed to connect to ROS master
```

**解决方案**：

```bash
# 1. 启动ROS核心
roscore &

# 2. 设置ROS环境
source /opt/ros/noetic/setup.bash

# 3. 检查ROS节点
rosnode list

# 4. 启动UR驱动
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.100
```

### 2. 控制问题

#### 问题：机械臂无响应

**症状**：
```
⚠️ 机械臂无响应
Joint positions not updated
```

**可能原因**：
- 机械臂处于急停状态
- 关节超出限制
- 控制命令被拒绝
- 安全约束触发

**解决方案**：

```python
# 1. 检查机械臂状态
def check_robot_status(robot):
    """检查机械臂状态"""
    try:
        # 检查急停状态
        if robot.is_emergency_stopped():
            print("⚠️ 机械臂处于急停状态")
            return False
            
        # 检查关节限制
        joint_pos = robot.get_joint_positions()
        joint_limits = robot.get_joint_limits()
        
        for i, (pos, limits) in enumerate(zip(joint_pos, joint_limits)):
            if pos < limits[0] or pos > limits[1]:
                print(f"⚠️ 关节{i}超出限制: {pos} (范围: {limits})")
                return False
                
        # 检查安全约束
        if not robot.check_safety_constraints():
            print("⚠️ 安全约束违反")
            return False
            
        return True
        
    except Exception as e:
        print(f"状态检查失败: {e}")
        return False

# 2. 重置机械臂状态
def reset_robot_state(robot):
    """重置机械臂状态"""
    try:
        # 清除急停
        robot.clear_emergency_stop()
        
        # 重置关节位置
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robot.move_to_joint_positions(home_position, blocking=True)
        
        print("✅ 机械臂状态已重置")
        return True
        
    except Exception as e:
        print(f"重置失败: {e}")
        return False
```

#### 问题：运动学求解失败

**症状**：
```
❌ 逆运动学求解失败
IK solution not found
```

**解决方案**：

```python
# 1. 检查目标位姿是否可达
def check_target_reachability(target_position, target_orientation, robot):
    """检查目标位姿是否可达"""
    try:
        # 获取工作空间边界
        workspace_bounds = robot.get_workspace_bounds()
        
        # 检查位置是否在工作空间内
        if not is_position_in_bounds(target_position, workspace_bounds):
            print(f"⚠️ 目标位置超出工作空间: {target_position}")
            return False
            
        # 检查姿态是否合理
        if not is_orientation_valid(target_orientation):
            print(f"⚠️ 目标姿态不合理: {target_orientation}")
            return False
            
        return True
        
    except Exception as e:
        print(f"可达性检查失败: {e}")
        return False

# 2. 调整IK求解参数
def adjust_ik_parameters(ik_solver):
    """调整IK求解参数"""
    # 增加最大迭代次数
    ik_solver.set_max_iterations(200)
    
    # 放宽收敛容差
    ik_solver.set_tolerance(1e-4)
    
    # 调整阻尼因子
    ik_solver.set_damping_factor(1e-2)
    
    print("✅ IK求解参数已调整")

# 3. 使用备用求解器
def use_backup_ik_solver(target_position, target_orientation, robot):
    """使用备用IK求解器"""
    try:
        # 尝试数值求解器
        ik_solver = NumericalIKSolver(robot)
        solution = ik_solver.solve(target_position, target_orientation)
        
        if solution is not None:
            return solution
            
        # 尝试解析求解器
        ik_solver = AnalyticalIKSolver(robot)
        solution = ik_solver.solve(target_position, target_orientation)
        
        return solution
        
    except Exception as e:
        print(f"备用求解器失败: {e}")
        return None
```

### 3. 性能问题

#### 问题：控制频率过低

**症状**：
```
⚠️ 控制频率过低: 25.0 Hz (目标: 50.0 Hz)
```

**解决方案**：

```python
# 1. 优化控制循环
def optimize_control_loop(controller):
    """优化控制循环性能"""
    # 减少不必要的计算
    controller.set_optimization_level("high")
    
    # 使用更高效的数据结构
    controller.use_numpy_arrays()
    
    # 启用并行计算
    controller.enable_parallel_processing()
    
    # 调整缓冲区大小
    controller.set_buffer_size(1024)

# 2. 系统性能优化
def optimize_system_performance():
    """优化系统性能"""
    import os
    
    # 设置CPU亲和性
    os.sched_setaffinity(0, {0, 1, 2, 3})
    
    # 设置实时优先级
    os.nice(-10)
    
    # 禁用CPU频率调节
    os.system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")

# 3. 网络优化
def optimize_network_performance():
    """优化网络性能"""
    # 增加网络缓冲区
    os.system("echo 16777216 | sudo tee /proc/sys/net/core/rmem_max")
    os.system("echo 16777216 | sudo tee /proc/sys/net/core/wmem_max")
    
    # 禁用Nagle算法
    os.system("echo 1 | sudo tee /proc/sys/net/ipv4/tcp_nodelay")
```

#### 问题：内存使用过高

**症状**：
```
⚠️ 内存使用率过高: 85%
```

**解决方案**：

```python
# 1. 内存监控
def monitor_memory_usage():
    """监控内存使用"""
    import psutil
    
    memory = psutil.virtual_memory()
    print(f"内存使用率: {memory.percent}%")
    print(f"可用内存: {memory.available / 1024**3:.2f} GB")
    
    if memory.percent > 80:
        print("⚠️ 内存使用率过高，建议优化")

# 2. 内存优化
def optimize_memory_usage():
    """优化内存使用"""
    import gc
    
    # 强制垃圾回收
    gc.collect()
    
    # 清理缓存
    import numpy as np
    np.clear_cache()
    
    # 减少数据记录频率
    controller.set_logging_frequency(10.0)  # 降低到10Hz

# 3. 数据压缩
def enable_data_compression():
    """启用数据压缩"""
    # 启用日志压缩
    logging_config = {
        'compression': True,
        'compression_level': 6,
        'max_file_size': '50MB'
    }
    controller.set_logging_config(logging_config)
```

### 4. 安全约束问题

#### 问题：安全约束违反

**症状**：
```
🚨 安全约束违反: 关节速度超出限制
```

**解决方案**：

```python
# 1. 检查安全约束
def check_safety_violations(robot, safety_manager):
    """检查安全约束违反"""
    violations = []
    
    # 检查关节限制
    joint_pos = robot.get_joint_positions()
    joint_limits = safety_manager.get_joint_limits()
    
    for i, (pos, limits) in enumerate(zip(joint_pos, joint_limits)):
        if pos < limits[0] or pos > limits[1]:
            violations.append(f"关节{i}超出限制: {pos}")
    
    # 检查速度限制
    joint_vel = robot.get_joint_velocities()
    vel_limits = safety_manager.get_velocity_limits()
    
    for i, (vel, limit) in enumerate(zip(joint_vel, vel_limits)):
        if abs(vel) > limit:
            violations.append(f"关节{i}速度超出限制: {vel}")
    
    return violations

# 2. 安全模式恢复
def enter_safe_mode(robot):
    """进入安全模式"""
    try:
        # 停止所有运动
        robot.stop_motion()
        
        # 缓慢移动到安全位置
        safe_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robot.move_to_joint_positions_slow(safe_position)
        
        # 启用安全监控
        robot.enable_safety_monitoring()
        
        print("✅ 已进入安全模式")
        return True
        
    except Exception as e:
        print(f"进入安全模式失败: {e}")
        return False

# 3. 调整安全参数
def adjust_safety_parameters(safety_manager):
    """调整安全参数"""
    # 临时放宽限制
    safety_manager.set_velocity_limits(max_velocity=0.3)  # 降低速度限制
    safety_manager.set_acceleration_limits(max_acceleration=1.0)  # 降低加速度限制
    
    # 增加安全边距
    safety_manager.set_safety_margin(0.1)
    
    print("✅ 安全参数已调整")
```

### 5. 数据记录问题

#### 问题：日志文件过大

**症状**：
```
⚠️ 日志文件过大: 2.5 GB
```

**解决方案**：

```python
# 1. 日志文件管理
def manage_log_files():
    """管理日志文件"""
    import os
    from pathlib import Path
    
    log_dir = Path("./logs")
    max_size = 100 * 1024 * 1024  # 100MB
    
    for log_file in log_dir.glob("*.log"):
        if log_file.stat().st_size > max_size:
            # 压缩旧日志
            os.system(f"gzip {log_file}")
            print(f"已压缩: {log_file}")

# 2. 自动清理
def auto_cleanup_logs():
    """自动清理日志"""
    import os
    from datetime import datetime, timedelta
    
    log_dir = "./logs"
    max_days = 7
    
    cutoff_date = datetime.now() - timedelta(days=max_days)
    
    for file in os.listdir(log_dir):
        file_path = os.path.join(log_dir, file)
        file_time = datetime.fromtimestamp(os.path.getctime(file_path))
        
        if file_time < cutoff_date:
            os.remove(file_path)
            print(f"已删除旧日志: {file}")

# 3. 日志轮转配置
def setup_log_rotation():
    """设置日志轮转"""
    log_config = {
        'max_file_size': '50MB',
        'backup_count': 5,
        'compression': True,
        'rotation': 'daily'
    }
    
    controller.set_logging_config(log_config)
```

## 🔧 调试工具

### 1. 系统诊断脚本

```python
#!/usr/bin/env python3
"""
系统诊断脚本
"""

import sys
import subprocess
import psutil
import numpy as np

def run_system_diagnosis():
    """运行系统诊断"""
    print("=== 系统诊断报告 ===")
    
    # 1. 系统信息
    print("\n1. 系统信息:")
    print(f"操作系统: {sys.platform}")
    print(f"Python版本: {sys.version}")
    print(f"CPU核心数: {psutil.cpu_count()}")
    print(f"内存总量: {psutil.virtual_memory().total / 1024**3:.2f} GB")
    
    # 2. 网络状态
    print("\n2. 网络状态:")
    try:
        result = subprocess.run(['ping', '-c', '3', '192.168.123.161'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ 主机械臂网络连接正常")
        else:
            print("❌ 主机械臂网络连接失败")
    except Exception as e:
        print(f"网络检查失败: {e}")
    
    # 3. 依赖检查
    print("\n3. 依赖检查:")
    required_packages = ['numpy', 'scipy', 'mujoco', 'unitree_sdk2py']
    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package} 已安装")
        except ImportError:
            print(f"❌ {package} 未安装")
    
    # 4. 性能检查
    print("\n4. 性能检查:")
    cpu_percent = psutil.cpu_percent(interval=1)
    memory_percent = psutil.virtual_memory().percent
    
    print(f"CPU使用率: {cpu_percent}%")
    print(f"内存使用率: {memory_percent}%")
    
    if cpu_percent > 80:
        print("⚠️ CPU使用率过高")
    if memory_percent > 80:
        print("⚠️ 内存使用率过高")

if __name__ == "__main__":
    run_system_diagnosis()
```

### 2. 性能监控脚本

```python
#!/usr/bin/env python3
"""
性能监控脚本
"""

import time
import threading
import psutil
import numpy as np

class PerformanceMonitor:
    def __init__(self):
        self.monitoring = False
        self.metrics = []
        
    def start_monitoring(self):
        """开始监控"""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.start()
        
    def stop_monitoring(self):
        """停止监控"""
        self.monitoring = False
        self.monitor_thread.join()
        
    def _monitor_loop(self):
        """监控循环"""
        while self.monitoring:
            metrics = {
                'timestamp': time.time(),
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': psutil.virtual_memory().percent,
                'network_io': psutil.net_io_counters()
            }
            
            self.metrics.append(metrics)
            time.sleep(1.0)
            
    def get_performance_report(self):
        """获取性能报告"""
        if not self.metrics:
            return "无监控数据"
            
        cpu_values = [m['cpu_percent'] for m in self.metrics]
        memory_values = [m['memory_percent'] for m in self.metrics]
        
        report = f"""
性能监控报告:
- 监控时长: {len(self.metrics)} 秒
- CPU使用率: 平均 {np.mean(cpu_values):.1f}%, 最大 {np.max(cpu_values):.1f}%
- 内存使用率: 平均 {np.mean(memory_values):.1f}%, 最大 {np.max(memory_values):.1f}%
        """
        
        return report

# 使用示例
monitor = PerformanceMonitor()
monitor.start_monitoring()

# 运行一段时间后
time.sleep(60)
monitor.stop_monitoring()
print(monitor.get_performance_report())
```

## 📞 技术支持

### 1. 收集诊断信息

```bash
#!/bin/bash
# 收集诊断信息脚本

echo "=== 收集诊断信息 ==="

# 系统信息
echo "系统信息:" > diagnosis.txt
uname -a >> diagnosis.txt
echo "" >> diagnosis.txt

# Python环境
echo "Python环境:" >> diagnosis.txt
python --version >> diagnosis.txt
pip list >> diagnosis.txt
echo "" >> diagnosis.txt

# 网络状态
echo "网络状态:" >> diagnosis.txt
ip addr show >> diagnosis.txt
ping -c 3 192.168.123.161 >> diagnosis.txt
echo "" >> diagnosis.txt

# 系统资源
echo "系统资源:" >> diagnosis.txt
free -h >> diagnosis.txt
df -h >> diagnosis.txt
echo "" >> diagnosis.txt

# 进程状态
echo "进程状态:" >> diagnosis.txt
ps aux | grep python >> diagnosis.txt
echo "" >> diagnosis.txt

echo "诊断信息已保存到 diagnosis.txt"
```

### 2. 联系技术支持

当遇到无法解决的问题时，请提供以下信息：

1. **系统信息**：
   - 操作系统版本
   - Python版本
   - 硬件配置

2. **错误信息**：
   - 完整的错误日志
   - 错误发生时的操作步骤
   - 错误发生前的系统状态

3. **配置文件**：
   - 相关的配置文件内容
   - 配置文件的修改历史

4. **诊断报告**：
   - 运行诊断脚本的结果
   - 性能监控数据

---

**📝 故障排除建议**:
- 保持系统日志的完整性
- 定期备份重要配置
- 建立问题记录和解决方案库
- 在安全环境下测试解决方案
- 及时更新系统和依赖包 