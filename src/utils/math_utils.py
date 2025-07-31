"""
数学工具函数模块
提供常用的数学计算功能
"""

import numpy as np
from typing import Tuple, List, Optional
import transforms3d as t3d


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    归一化四元数
    
    Args:
        q: 输入四元数
        
    Returns:
        归一化后的四元数
    """
    return q / np.linalg.norm(q)


def quaternion_to_euler(q: np.ndarray, order: str = 'sxyz') -> np.ndarray:
    """
    四元数转欧拉角
    
    Args:
        q: 四元数 [w, x, y, z]
        order: 欧拉角顺序 ('sxyz', 'szyx', etc.)
        
    Returns:
        欧拉角 [roll, pitch, yaw] (弧度)
    """
    return t3d.euler.quat2euler(q, order)


def euler_to_quaternion(euler: np.ndarray, order: str = 'sxyz') -> np.ndarray:
    """
    欧拉角转四元数
    
    Args:
        euler: 欧拉角 [roll, pitch, yaw] (弧度)
        order: 欧拉角顺序
        
    Returns:
        四元数 [w, x, y, z]
    """
    return t3d.euler.euler2quat(euler[0], euler[1], euler[2], order)


def rotation_matrix_to_euler(R: np.ndarray, order: str = 'sxyz') -> np.ndarray:
    """
    旋转矩阵转欧拉角
    
    Args:
        R: 3x3旋转矩阵
        order: 欧拉角顺序
        
    Returns:
        欧拉角 [roll, pitch, yaw] (弧度)
    """
    return t3d.euler.mat2euler(R, order)


def euler_to_rotation_matrix(euler: np.ndarray, order: str = 'sxyz') -> np.ndarray:
    """
    欧拉角转旋转矩阵
    
    Args:
        euler: 欧拉角 [roll, pitch, yaw] (弧度)
        order: 欧拉角顺序
        
    Returns:
        3x3旋转矩阵
    """
    return t3d.euler.euler2mat(euler[0], euler[1], euler[2], order)


def quaternion_distance(q1: np.ndarray, q2: np.ndarray) -> float:
    """
    计算两个四元数之间的距离
    
    Args:
        q1: 第一个四元数
        q2: 第二个四元数
        
    Returns:
        四元数距离
    """
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    
    # 计算点积
    dot_product = np.dot(q1, q2)
    
    # 处理四元数方向
    if dot_product < 0:
        q2 = -q2
        dot_product = -dot_product
    
    # 计算角度距离
    angle = 2 * np.arccos(np.clip(dot_product, -1, 1))
    return angle


def pose_distance(pose1: np.ndarray, pose2: np.ndarray, 
                 position_weight: float = 1.0, orientation_weight: float = 1.0) -> float:
    """
    计算两个位姿之间的距离
    
    Args:
        pose1: 第一个位姿 [x, y, z, qw, qx, qy, qz]
        pose2: 第二个位姿 [x, y, z, qw, qx, qy, qz]
        position_weight: 位置权重
        orientation_weight: 姿态权重
        
    Returns:
        位姿距离
    """
    # 位置距离
    position1 = pose1[:3]
    position2 = pose2[:3]
    position_dist = np.linalg.norm(position1 - position2)
    
    # 姿态距离
    orientation1 = pose1[3:]
    orientation2 = pose2[3:]
    orientation_dist = quaternion_distance(orientation1, orientation2)
    
    # 加权组合
    total_distance = position_weight * position_dist + orientation_weight * orientation_dist
    
    return total_distance


def interpolate_pose(pose1: np.ndarray, pose2: np.ndarray, t: float) -> np.ndarray:
    """
    位姿插值
    
    Args:
        pose1: 起始位姿
        pose2: 结束位姿
        t: 插值参数 (0-1)
        
    Returns:
        插值后的位姿
    """
    # 位置线性插值
    position1 = pose1[:3]
    position2 = pose2[:3]
    interpolated_position = (1 - t) * position1 + t * position2
    
    # 姿态球面线性插值
    orientation1 = pose1[3:]
    orientation2 = pose2[3:]
    
    # 确保四元数为单位四元数
    orientation1 = normalize_quaternion(orientation1)
    orientation2 = normalize_quaternion(orientation2)
    
    # 计算点积
    dot_product = np.dot(orientation1, orientation2)
    
    # 处理四元数方向
    if dot_product < 0:
        orientation2 = -orientation2
        dot_product = -dot_product
    
    # 如果四元数接近，使用线性插值
    if dot_product > 0.9995:
        interpolated_orientation = orientation1 + t * (orientation2 - orientation1)
    else:
        # 使用SLERP
        theta = np.arccos(dot_product)
        sin_theta = np.sin(theta)
        
        if sin_theta > 1e-6:
            w1 = np.sin((1 - t) * theta) / sin_theta
            w2 = np.sin(t * theta) / sin_theta
            interpolated_orientation = w1 * orientation1 + w2 * orientation2
        else:
            interpolated_orientation = orientation1
    
    interpolated_orientation = normalize_quaternion(interpolated_orientation)
    
    return np.concatenate([interpolated_position, interpolated_orientation])


def transform_point(point: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """
    变换点坐标
    
    Args:
        point: 3D点 [x, y, z]
        transform: 4x4变换矩阵
        
    Returns:
        变换后的点
    """
    point_homogeneous = np.append(point, 1)
    transformed_point_homogeneous = transform @ point_homogeneous
    return transformed_point_homogeneous[:3]


def transform_pose(pose: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """
    变换位姿
    
    Args:
        pose: 位姿 [x, y, z, qw, qx, qy, qz]
        transform: 4x4变换矩阵
        
    Returns:
        变换后的位姿
    """
    # 变换位置
    position = pose[:3]
    transformed_position = transform_point(position, transform)
    
    # 变换姿态
    orientation = pose[3:]
    rotation_matrix = transform[:3, :3]
    
    # 将四元数转换为旋转矩阵
    orientation_matrix = t3d.quaternions.quat2mat(orientation)
    
    # 组合旋转
    combined_rotation = rotation_matrix @ orientation_matrix
    
    # 转换回四元数
    transformed_orientation = t3d.quaternions.mat2quat(combined_rotation)
    
    return np.concatenate([transformed_position, transformed_orientation])


def create_transform_matrix(position: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    """
    创建变换矩阵
    
    Args:
        position: 位置 [x, y, z]
        orientation: 姿态四元数 [w, x, y, z]
        
    Returns:
        4x4变换矩阵
    """
    transform = np.eye(4)
    transform[:3, :3] = t3d.quaternions.quat2mat(orientation)
    transform[:3, 3] = position
    return transform


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    """
    计算变换矩阵的逆
    
    Args:
        transform: 4x4变换矩阵
        
    Returns:
        逆变换矩阵
    """
    R = transform[:3, :3]
    t = transform[:3, 3]
    
    inverse_transform = np.eye(4)
    inverse_transform[:3, :3] = R.T
    inverse_transform[:3, 3] = -R.T @ t
    
    return inverse_transform


def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
    """
    计算两个向量之间的角度
    
    Args:
        v1: 第一个向量
        v2: 第二个向量
        
    Returns:
        角度（弧度）
    """
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_angle = np.clip(cos_angle, -1, 1)
    return np.arccos(cos_angle)


def cross_product_matrix(v: np.ndarray) -> np.ndarray:
    """
    计算向量的叉积矩阵
    
    Args:
        v: 3D向量
        
    Returns:
        3x3叉积矩阵
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def skew_symmetric_matrix(v: np.ndarray) -> np.ndarray:
    """
    计算向量的斜对称矩阵
    
    Args:
        v: 3D向量
        
    Returns:
        3x3斜对称矩阵
    """
    return cross_product_matrix(v)


def exponential_map(omega: np.ndarray, theta: float) -> np.ndarray:
    """
    指数映射（Rodrigues公式）
    
    Args:
        omega: 旋转轴（单位向量）
        theta: 旋转角度
        
    Returns:
        3x3旋转矩阵
    """
    omega_hat = skew_symmetric_matrix(omega)
    I = np.eye(3)
    
    R = I + np.sin(theta) * omega_hat + (1 - np.cos(theta)) * (omega_hat @ omega_hat)
    return R


def logarithmic_map(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    对数映射
    
    Args:
        R: 3x3旋转矩阵
        
    Returns:
        omega: 旋转轴
        theta: 旋转角度
    """
    trace = np.trace(R)
    theta = np.arccos(np.clip((trace - 1) / 2, -1, 1))
    
    if np.abs(theta) < 1e-6:
        omega = np.zeros(3)
    else:
        omega_hat = (R - R.T) / (2 * np.sin(theta))
        omega = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
    
    return omega, theta 