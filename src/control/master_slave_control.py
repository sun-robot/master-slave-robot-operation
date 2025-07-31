"""
主从控制模块
实现机械臂之间的协调控制
"""

import numpy as np
from typing import List, Tuple, Optional, Callable
import time
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics
from ..robot_models.d1arm_robot import D1ArmModel, D1ArmSimulator


class MasterSlaveController:
    """主从控制器"""
    
    def __init__(self, master_model: D1ArmModel, slave_model: D1ArmModel):
        """
        初始化主从控制器
        
        Args:
            master_model: 主机械臂模型
            slave_model: 从机械臂模型
        """
        self.master_model = master_model
        self.slave_model = slave_model
        
        # 创建运动学求解器
        self.master_fk = ForwardKinematics(master_model.get_dh_params())
        self.slave_fk = ForwardKinematics(slave_model.get_dh_params())
        
        self.master_ik = InverseKinematics(
            self.master_fk, 
            master_model.get_joint_limits()
        )
        self.slave_ik = InverseKinematics(
            self.slave_fk, 
            slave_model.get_joint_limits()
        )
        
        # 创建仿真器
        self.master_simulator = D1ArmSimulator(master_model)
        self.slave_simulator = D1ArmSimulator(slave_model)
        
        # 控制参数
        self.control_frequency = 100.0  # Hz
        self.control_dt = 1.0 / self.control_frequency
        
        # 状态变量
        self.is_running = False
        self.master_target_angles = None
        self.slave_target_angles = None
        
        # 控制模式
        self.control_mode = 'position'  # 'position', 'velocity', 'force'
        
        # 变换矩阵（主从之间的坐标变换）
        self.master_to_slave_transform = np.eye(4)
        
    def set_master_to_slave_transform(self, transform: np.ndarray):
        """
        设置主从机械臂之间的坐标变换
        
        Args:
            transform: 4x4变换矩阵
        """
        self.master_to_slave_transform = transform.copy()
    
    def set_control_mode(self, mode: str):
        """
        设置控制模式
        
        Args:
            mode: 控制模式 ('position', 'velocity', 'force')
        """
        if mode in ['position', 'velocity', 'force']:
            self.control_mode = mode
        else:
            raise ValueError(f"不支持的控制模式: {mode}")
    
    def start_control(self):
        """启动主从控制"""
        self.is_running = True
        print("主从控制已启动")
    
    def stop_control(self):
        """停止主从控制"""
        self.is_running = False
        print("主从控制已停止")
    
    def update_master_joint_angles(self, joint_angles: np.ndarray):
        """
        更新主机械臂关节角度
        
        Args:
            joint_angles: 主机械臂关节角度
        """
        if self.master_model.check_joint_limits(joint_angles):
            self.master_simulator.set_joint_angles(joint_angles)
            self.master_target_angles = joint_angles.copy()
        else:
            raise ValueError("主机械臂关节角度超出限位范围")
    
    def get_slave_joint_angles(self) -> np.ndarray:
        """获取从机械臂关节角度"""
        return self.slave_simulator.get_current_joint_angles()
    
    def calculate_slave_target(self, master_position: np.ndarray, 
                             master_orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        根据主机械臂位姿计算从机械臂目标位姿
        
        Args:
            master_position: 主机械臂末端位置
            master_orientation: 主机械臂末端姿态
            
        Returns:
            slave_target_position: 从机械臂目标位置
            slave_target_orientation: 从机械臂目标姿态
        """
        # 应用坐标变换
        master_pose = np.eye(4)
        master_pose[:3, :3] = t3d.quaternions.quat2mat(master_orientation)
        master_pose[:3, 3] = master_position
        
        # 计算变换后的位姿
        transformed_pose = self.master_to_slave_transform @ master_pose
        
        # 提取从机械臂目标位姿
        slave_target_position = transformed_pose[:3, 3]
        slave_target_orientation = t3d.quaternions.mat2quat(transformed_pose[:3, :3])
        
        return slave_target_position, slave_target_orientation
    
    def control_cycle(self):
        """执行一个控制周期"""
        if not self.is_running:
            return
        
        try:
            # 获取主机械臂当前位姿
            master_angles = self.master_simulator.get_current_joint_angles()
            master_position, master_orientation = self.master_fk.forward_kinematics(master_angles)
            
            # 计算从机械臂目标位姿
            slave_target_position, slave_target_orientation = self.calculate_slave_target(
                master_position, master_orientation)
            
            # 求解从机械臂逆运动学
            slave_current_angles = self.slave_simulator.get_current_joint_angles()
            
            # 使用优化方法求解IK
            slave_target_angles, success = self.slave_ik.ik_optimization_method(
                slave_target_position, slave_target_orientation, slave_current_angles)
            
            if success:
                # 更新从机械臂目标角度
                self.slave_target_angles = slave_target_angles
                
                # 仿真从机械臂运动
                self.slave_simulator.simulate_motion(slave_target_angles, self.control_dt)
                
                # 输出调试信息
                print(f"主机械臂位姿: 位置={master_position}, 姿态={master_orientation}")
                print(f"从机械臂目标位姿: 位置={slave_target_position}, 姿态={slave_target_orientation}")
                print(f"从机械臂关节角度: {slave_target_angles}")
                
            else:
                print("警告: 从机械臂逆运动学求解失败")
                
        except Exception as e:
            print(f"控制周期执行错误: {e}")
    
    def run_control_loop(self, duration: float = 10.0):
        """
        运行控制循环
        
        Args:
            duration: 运行时间（秒）
        """
        self.start_control()
        
        start_time = time.time()
        while self.is_running and (time.time() - start_time) < duration:
            self.control_cycle()
            time.sleep(self.control_dt)
        
        self.stop_control()
    
    def get_control_status(self) -> dict:
        """获取控制状态"""
        return {
            'is_running': self.is_running,
            'control_mode': self.control_mode,
            'control_frequency': self.control_frequency,
            'master_angles': self.master_simulator.get_current_joint_angles(),
            'slave_angles': self.slave_simulator.get_current_joint_angles(),
            'master_velocities': self.master_simulator.get_current_joint_velocities(),
            'slave_velocities': self.slave_simulator.get_current_joint_velocities()
        }


class ForceReflectionController(MasterSlaveController):
    """力反馈主从控制器"""
    
    def __init__(self, master_model: D1ArmModel, slave_model: D1ArmModel):
        super().__init__(master_model, slave_model)
        
        # 力反馈参数
        self.force_scaling = 1.0
        self.velocity_scaling = 1.0
        
        # 力传感器数据（仿真）
        self.slave_force_sensor = np.zeros(6)  # [fx, fy, fz, mx, my, mz]
        
    def set_force_scaling(self, scaling: float):
        """设置力反馈缩放系数"""
        self.force_scaling = scaling
    
    def set_velocity_scaling(self, scaling: float):
        """设置速度缩放系数"""
        self.velocity_scaling = scaling
    
    def update_slave_force_sensor(self, force_data: np.ndarray):
        """更新从机械臂力传感器数据"""
        self.slave_force_sensor = force_data.copy()
    
    def calculate_master_force_feedback(self) -> np.ndarray:
        """计算主机械臂的力反馈"""
        # 将力传感器数据转换到主机械臂坐标系
        transformed_force = self.master_to_slave_transform.T @ self.slave_force_sensor
        return transformed_force * self.force_scaling
    
    def control_cycle(self):
        """执行力反馈控制周期"""
        if not self.is_running:
            return
        
        try:
            # 获取主机械臂当前位姿
            master_angles = self.master_simulator.get_current_joint_angles()
            master_position, master_orientation = self.master_fk.forward_kinematics(master_angles)
            
            # 计算从机械臂目标位姿（考虑力反馈）
            slave_target_position, slave_target_orientation = self.calculate_slave_target(
                master_position, master_orientation)
            
            # 应用力反馈修正
            force_feedback = self.calculate_master_force_feedback()
            # 这里可以添加基于力反馈的位姿修正逻辑
            
            # 求解从机械臂逆运动学
            slave_current_angles = self.slave_simulator.get_current_joint_angles()
            slave_target_angles, success = self.slave_ik.ik_optimization_method(
                slave_target_position, slave_target_orientation, slave_current_angles)
            
            if success:
                self.slave_target_angles = slave_target_angles
                self.slave_simulator.simulate_motion(slave_target_angles, self.control_dt)
                
                # 输出力反馈信息
                print(f"力传感器数据: {self.slave_force_sensor}")
                print(f"主机械臂力反馈: {force_feedback}")
                
            else:
                print("警告: 从机械臂逆运动学求解失败")
                
        except Exception as e:
            print(f"力反馈控制周期执行错误: {e}")


# 导入必要的模块
import transforms3d as t3d 