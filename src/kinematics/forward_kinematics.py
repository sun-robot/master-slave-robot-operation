"""
正向运动学模块
实现关节角度到末端位姿的转换
"""

import numpy as np
from typing import List, Tuple, Optional
import transforms3d as t3d


class ForwardKinematics:
    """正向运动学计算类"""
    
    def __init__(self, dh_params: np.ndarray):
        """
        初始化正向运动学
        
        Args:
            dh_params: DH参数矩阵，形状为(n_joints, 4)
                      每行包含 [a, alpha, d, theta]
        """
        self.dh_params = dh_params
        self.n_joints = len(dh_params)
        
    def dh_transform(self, a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """
        计算单个关节的DH变换矩阵
        
        Args:
            a: 连杆长度
            alpha: 连杆扭转角
            d: 连杆偏移
            theta: 关节角
            
        Returns:
            4x4变换矩阵
        """
        # 计算DH变换矩阵
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        T = np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算正向运动学
        
        Args:
            joint_angles: 关节角度数组，形状为(n_joints,)
            
        Returns:
            position: 末端位置 (x, y, z)
            orientation: 末端姿态 (四元数或旋转矩阵)
        """
        if len(joint_angles) != self.n_joints:
            raise ValueError(f"关节角度数量({len(joint_angles)})与DH参数数量({self.n_joints})不匹配")
        
        # 计算总变换矩阵
        T_total = np.eye(4)
        
        for i in range(self.n_joints):
            a, alpha, d, theta = self.dh_params[i]
            # 使用关节角度更新theta
            theta += joint_angles[i]
            
            T_i = self.dh_transform(a, alpha, d, theta)
            T_total = T_total @ T_i
        
        # 提取位置和姿态
        position = T_total[:3, 3]
        rotation_matrix = T_total[:3, :3]
        
        # 转换为四元数
        orientation = t3d.quaternions.mat2quat(rotation_matrix)
        
        return position, orientation
    
    def get_joint_positions(self, joint_angles: np.ndarray) -> List[np.ndarray]:
        """
        获取所有关节的位置
        
        Args:
            joint_angles: 关节角度数组
            
        Returns:
            所有关节位置的列表
        """
        positions = []
        T_total = np.eye(4)
        
        for i in range(self.n_joints):
            a, alpha, d, theta = self.dh_params[i]
            theta += joint_angles[i]
            
            T_i = self.dh_transform(a, alpha, d, theta)
            T_total = T_total @ T_i
            
            # 记录当前关节位置
            joint_pos = T_total[:3, 3]
            positions.append(joint_pos)
        
        return positions
    
    def jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        计算雅可比矩阵
        
        Args:
            joint_angles: 关节角度数组
            
        Returns:
            6xn雅可比矩阵
        """
        epsilon = 1e-6
        n_joints = len(joint_angles)
        J = np.zeros((6, n_joints))
        
        # 计算当前位姿
        pos_current, orient_current = self.forward_kinematics(joint_angles)
        
        for i in range(n_joints):
            # 扰动第i个关节
            joint_angles_perturbed = joint_angles.copy()
            joint_angles_perturbed[i] += epsilon
            
            # 计算扰动后的位姿
            pos_perturbed, orient_perturbed = self.forward_kinematics(joint_angles_perturbed)
            
            # 计算位置雅可比
            J[:3, i] = (pos_perturbed - pos_current) / epsilon
            
            # 计算姿态雅可比（简化处理，只考虑前3个分量）
            orientation_error = orient_perturbed - orient_current
            J[3:, i] = orientation_error[:3] / epsilon
        
        return J 