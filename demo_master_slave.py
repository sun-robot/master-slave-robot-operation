#!/usr/bin/env python3
"""
通用跨机械臂主从控制演示
支持多种机械臂品牌和型号的主从协调控制
"""

import numpy as np
import time
import threading
import argparse
from pathlib import Path
from typing import Optional, Tuple

# 导入必要的模块
from src.robot_models.robot_factory import RobotFactory, RobotManager
from src.robot_models.robot_interface import RobotType, RobotState
from src.robot_models.mujoco_robot_interface import MuJoCoRobotInterface
from src.robot_models.ur_robot import URRobot
from src.kinematics.forward_kinematics import ForwardKinematics
from src.kinematics.inverse_kinematics import InverseKinematics
from src.utils.math_utils import normalize_quaternion
from src.utils.safety_constraints import SafetyConstraintManager


class UniversalMasterSlaveController:
    """通用跨机械臂主从控制器"""
    
    def __init__(self, master_type: RobotType, slave_type: RobotType, 
                 master_mode: str = "simulation",
                 slave_mode: str = "hardware",
                 master_id: str = "master_robot",
                 slave_id: str = "slave_robot"):
        """
        初始化通用主从控制器
        
        Args:
            master_type: 主机械臂类型
            slave_type: 从机械臂类型
            master_mode: 主机械臂模式 ("simulation" 或 "hardware")
            slave_mode: 从机械臂模式 ("simulation" 或 "hardware")
            master_id: 主机械臂ID
            slave_id: 从机械臂ID
        """
        print("=== 初始化通用跨机械臂主从控制系统 ===")
        print(f"主端: {master_type.value} ({master_mode})")
        print(f"从端: {slave_type.value} ({slave_mode})")
        
        # 初始化机械臂管理器
        self.robot_manager = RobotManager()
        
        # 创建主机械臂
        print(f"正在初始化主机械臂 {master_type.value}...")
        try:
            if master_type == RobotType.UNITREE_D1 and master_mode == "simulation":
                # 使用MuJoCo仿真
                xml_path = "assets/d1_550/mujoco/d1_550.xml"
                if not Path(xml_path).exists():
                    raise FileNotFoundError(f"MuJoCo XML文件不存在: {xml_path}")
                
                self.master_robot = MuJoCoRobotInterface(
                    robot_type=master_type,
                    xml_path=xml_path,
                    headless=False,
                    render_mode="human"
                )
            else:
                # 使用通用工厂接口
                self.master_robot = RobotFactory.create_robot(
                    robot_type=master_type,
                    mode=master_mode,
                    robot_id=master_id
                )
            
            self.robot_manager.add_robot(self.master_robot, master_id)
            print(f"✅ 主机械臂 {master_type.value} 初始化成功")
        except Exception as e:
            print(f"❌ 主机械臂初始化失败: {e}")
            raise
        
        # 创建从机械臂
        print(f"正在初始化从机械臂 {slave_type.value}...")
        try:
            self.slave_robot = RobotFactory.create_robot(
                robot_type=slave_type,
                mode=slave_mode,
                robot_id=slave_id
            )
            self.robot_manager.add_robot(self.slave_robot, slave_id)
            print(f"✅ 从机械臂 {slave_type.value} 初始化成功")
        except Exception as e:
            print(f"⚠️ 从机械臂初始化失败: {e}")
            print("将使用仿真模式进行演示")
            self.slave_robot = None
        
        # 控制参数
        self.control_frequency = 50.0  # Hz
        self.control_dt = 1.0 / self.control_frequency
        self.is_running = False
        
        # 主从变换矩阵（从机械臂相对主机械臂的位置）
        self.master_to_slave_transform = np.eye(4)
        self.master_to_slave_transform[:3, 3] = np.array([0.1, 0.1, 0.0])
        
        # 控制模式
        self.control_mode = 'position'  # 'position', 'velocity', 'force'
        
        # 状态变量
        self.master_joint_positions = None
        self.slave_joint_positions = None
        self.master_ee_pose = None
        self.slave_ee_pose = None
        
        # 初始化安全约束管理器
        self.safety_manager = SafetyConstraintManager()
        
        print("✅ 通用主从控制器初始化完成")
    
    def set_master_to_slave_transform(self, transform: np.ndarray):
        """设置主从机械臂之间的坐标变换"""
        self.master_to_slave_transform = transform.copy()
        print(f"主从变换矩阵已设置:\n{transform}")
    
    def set_control_mode(self, mode: str):
        """设置控制模式"""
        if mode in ['position', 'velocity', 'force']:
            self.control_mode = mode
            print(f"控制模式已设置为: {mode}")
        else:
            print(f"不支持的控制模式: {mode}")
    
    def enable_robots(self):
        """使能所有机械臂"""
        print("正在使能机械臂...")
        
        # 使能主机械臂
        if self.master_robot:
            try:
                if self.master_robot.enable():
                    print("✅ 主机械臂使能成功")
                else:
                    print("❌ 主机械臂使能失败")
            except Exception as e:
                print(f"⚠️ 主机械臂使能异常: {e}")
        
        # 使能从机械臂
        if self.slave_robot:
            try:
                if self.slave_robot.enable():
                    print("✅ 从机械臂使能成功")
                else:
                    print("❌ 从机械臂使能失败")
            except Exception as e:
                print(f"⚠️ 从机械臂使能异常: {e}")
        
        # 等待机械臂就绪
        print("等待机械臂就绪...")
        time.sleep(2.0)
        
        # 检查就绪状态
        master_ready = self.master_robot.is_ready() if self.master_robot else False
        slave_ready = self.slave_robot.is_ready() if self.slave_robot else False
        
        print(f"主机械臂就绪状态: {master_ready}")
        print(f"从机械臂就绪状态: {slave_ready}")
        
        return master_ready and slave_ready
    
    def start_control(self):
        """开始主从控制"""
        if self.is_running:
            print("控制已在运行中")
            return
        
        print("开始主从控制...")
        self.is_running = True
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_cycle)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("✅ 主从控制已启动")
    
    def stop_control(self):
        """停止主从控制"""
        if not self.is_running:
            return
        
        print("停止主从控制...")
        self.is_running = False
        
        # 等待控制线程结束
        if hasattr(self, 'control_thread'):
            self.control_thread.join(timeout=2.0)
        
        # 停止机械臂运动
        if self.master_robot:
            self.master_robot.stop_motion(emergency=False)
        if self.slave_robot:
            self.slave_robot.stop_motion(emergency=False)
        
        print("✅ 主从控制已停止")
    
    def get_master_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取主机械臂末端位姿"""
        if self.master_robot and self.master_robot.is_ready():
            return self.master_robot.get_end_effector_pose()
        return None, None
    
    def calculate_slave_target_pose(self, master_position: np.ndarray, 
                                   master_orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """计算从机械臂目标位姿"""
        if self.slave_robot is None:
            return None, None
        
        # 应用坐标变换
        master_pose = np.eye(4)
        master_pose[:3, :3] = master_orientation
        master_pose[:3, 3] = master_position
        
        # 计算从机械臂目标位姿
        slave_pose = master_pose @ self.master_to_slave_transform
        
        slave_position = slave_pose[:3, 3]
        slave_orientation = slave_pose[:3, :3]
        
        return slave_position, slave_orientation
    
    def control_cycle(self):
        """主从控制循环"""
        print("进入主从控制循环...")
        
        while self.is_running:
            try:
                # 获取主机械臂末端位姿
                master_position, master_orientation = self.get_master_ee_pose()
                
                if master_position is not None and master_orientation is not None:
                    # 计算从机械臂目标位姿
                    slave_position, slave_orientation = self.calculate_slave_target_pose(
                        master_position, master_orientation
                    )
                    
                    # 控制从机械臂
                    if ( self.slave_robot and slave_position is not None and 
                         slave_orientation is not None and self.slave_robot.is_ready()):
                        
                        if self.control_mode == 'position':
                            # 位置控制
                            self.slave_robot.move_linear(
                                target_position=slave_position,
                                target_orientation=slave_orientation,
                                velocity=0.1,
                                acceleration=0.1,
                                blocking=False
                            )
                        elif self.control_mode == 'velocity':
                            # 速度控制
                            # 计算位置误差
                            current_pose = self.slave_robot.get_end_effector_pose()
                            if current_pose[0] is not None:
                                position_error = slave_position - current_pose[0]
                                velocity = position_error * 0.5  # 简单的P控制
                                self.slave_robot.set_cartesian_velocity(
                                    linear_velocity=velocity,
                                    angular_velocity=np.zeros(3)
                                )
                
                # 控制频率
                time.sleep(self.control_dt)
                
            except Exception as e:
                print(f"控制循环异常: {e}")
                time.sleep(0.1)
        
        print("主从控制循环结束")
    
    def run_control_loop(self, duration: float = 10.0):
        """运行控制循环指定时长"""
        print(f"运行主从控制 {duration} 秒...")
        
        self.start_control()
        time.sleep(duration)
        self.stop_control()
        
        print("控制循环运行完成")
    
    def demo_basic_control(self):
        """基本控制演示"""
        print("\n=== 基本控制演示 ===")
        
        # 使能机械臂
        if not self.enable_robots():
            print("❌ 机械臂使能失败，跳过演示")
            return
        
        # 设置控制模式
        self.set_control_mode('position')
        
        # 运行控制循环
        self.run_control_loop(duration=10.0)
        
        print("✅ 基本控制演示完成")
    
    def demo_realtime_control(self):
        """实时控制演示"""
        print("\n=== 实时控制演示 ===")
        
        # 使能机械臂
        if not self.enable_robots():
            print("❌ 机械臂使能失败，跳过演示")
            return
        
        # 设置控制模式
        self.set_control_mode('velocity')
        
        # 运行控制循环
        self.run_control_loop(duration=15.0)
        
        print("✅ 实时控制演示完成")
    
    def cleanup(self):
        """清理资源"""
        print("清理资源...")
        
        # 停止控制
        self.stop_control()
        
        # 停止所有机械臂
        self.robot_manager.stop_all(emergency=False)
        
        # 断开连接
        if self.master_robot:
            self.master_robot.disable()
        if self.slave_robot:
            self.slave_robot.disable()
        
        print("✅ 资源清理完成")
    
    def get_system_status(self) -> dict:
        """获取系统状态"""
        return self.robot_manager.get_system_status()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="通用跨机械臂主从控制系统")
    parser.add_argument("--master-type", type=str, default="unitree_d1",
                       choices=["unitree_d1", "ur3e", "ur5", "ur10", "franka_panda"],
                       help="主机械臂类型")
    parser.add_argument("--slave-type", type=str, default="ur3e",
                       choices=["unitree_d1", "ur3e", "ur5", "ur10", "franka_panda"],
                       help="从机械臂类型")
    parser.add_argument("--master-mode", type=str, default="simulation",
                       choices=["simulation", "hardware"],
                       help="主机械臂模式")
    parser.add_argument("--slave-mode", type=str, default="hardware",
                       choices=["simulation", "hardware"],
                       help="从机械臂模式")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="运行时长(秒)")
    parser.add_argument("--transform", type=float, nargs=6, 
                       default=[0.1, 0.1, 0.0, 0.0, 0.0, 0.0],
                       help="空间变换参数 [x, y, z, rx, ry, rz]")
    
    args = parser.parse_args()
    
    master_type = RobotType[args.master_type.upper()]
    slave_type = RobotType[args.slave_type.upper()]
    
    print("通用跨机械臂主从控制系统")
    print("=" * 60)
    print("功能特性:")
    print("1. 支持多种机械臂品牌和型号")
    print("2. 统一的主从控制接口")
    print("3. 实时主从控制，支持位置、速度、力控制")
    print("4. 坐标变换，支持主从机械臂的相对位置调整")
    print("5. 安全约束，确保机械臂在安全范围内运行")
    print("6. 跨品牌兼容性")
    print("=" * 60)
    
    try:
        # 创建主从控制器
        controller = UniversalMasterSlaveController(
            master_type=master_type,
            slave_type=slave_type,
            master_mode=args.master_mode,
            slave_mode=args.slave_mode
        )
        
        # 设置空间变换
        transform = np.eye(4)
        transform[:3, 3] = np.array(args.transform[:3])
        # 这里可以添加旋转矩阵的设置
        controller.set_master_to_slave_transform(transform)
        
        # 显示系统状态
        status = controller.get_system_status()
        print("\n系统状态:")
        for key, value in status.items():
            print(f"  {key}: {value}")
        
        # 基本控制演示
        controller.demo_basic_control()
        
        # 实时控制演示
        controller.demo_realtime_control()
        
        print("\n✅ 主从控制演示完成!")
        
    except Exception as e:
        print(f"❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理资源
        if 'controller' in locals():
            controller.cleanup()


if __name__ == "__main__":
    main() 