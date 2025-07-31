"""
轨迹规划模块
实现机械臂的路径规划和轨迹生成
"""

import numpy as np
from typing import List, Tuple, Optional, Callable
from scipy.interpolate import interp1d
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics


class TrajectoryPlanner:
    """轨迹规划器"""
    
    def __init__(self, fk_solver: ForwardKinematics, ik_solver: InverseKinematics):
        """
        初始化轨迹规划器
        
        Args:
            fk_solver: 正向运动学求解器
            ik_solver: 逆向运动学求解器
        """
        self.fk_solver = fk_solver
        self.ik_solver = ik_solver
        
    def linear_trajectory(self, start_angles: np.ndarray, end_angles: np.ndarray,
                         duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        生成线性关节轨迹
        
        Args:
            start_angles: 起始关节角度
            end_angles: 结束关节角度
            duration: 运动时间
            dt: 时间步长
            
        Returns:
            time_array: 时间数组
            trajectory: 关节角度轨迹
        """
        num_steps = int(duration / dt)
        time_array = np.linspace(0, duration, num_steps)
        
        # 线性插值
        trajectory = np.zeros((num_steps, len(start_angles)))
        for i in range(len(start_angles)):
            trajectory[:, i] = np.linspace(start_angles[i], end_angles[i], num_steps)
        
        return time_array, trajectory
    
    def cubic_trajectory(self, start_angles: np.ndarray, end_angles: np.ndarray,
                        start_velocities: np.ndarray, end_velocities: np.ndarray,
                        duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成三次多项式轨迹
        
        Args:
            start_angles: 起始关节角度
            end_angles: 结束关节角度
            start_velocities: 起始关节速度
            end_velocities: 结束关节速度
            duration: 运动时间
            dt: 时间步长
            
        Returns:
            time_array: 时间数组
            trajectory: 关节角度轨迹
            velocity_trajectory: 关节速度轨迹
        """
        num_steps = int(duration / dt)
        time_array = np.linspace(0, duration, num_steps)
        
        trajectory = np.zeros((num_steps, len(start_angles)))
        velocity_trajectory = np.zeros((num_steps, len(start_angles)))
        
        for i in range(len(start_angles)):
            # 三次多项式系数
            a0 = start_angles[i]
            a1 = start_velocities[i]
            a2 = (3 * (end_angles[i] - start_angles[i]) - 
                  2 * start_velocities[i] * duration - end_velocities[i] * duration) / (duration ** 2)
            a3 = (2 * (start_angles[i] - end_angles[i]) + 
                  (start_velocities[i] + end_velocities[i]) * duration) / (duration ** 3)
            
            # 计算轨迹
            t = time_array
            trajectory[:, i] = a0 + a1 * t + a2 * t**2 + a3 * t**3
            velocity_trajectory[:, i] = a1 + 2 * a2 * t + 3 * a3 * t**2
        
        return time_array, trajectory, velocity_trajectory
    
    def cartesian_trajectory(self, start_position: np.ndarray, end_position: np.ndarray,
                           start_orientation: np.ndarray, end_orientation: np.ndarray,
                           duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成笛卡尔空间轨迹
        
        Args:
            start_position: 起始位置
            end_position: 结束位置
            start_orientation: 起始姿态
            end_orientation: 结束姿态
            duration: 运动时间
            dt: 时间步长
            
        Returns:
            time_array: 时间数组
            position_trajectory: 位置轨迹
            orientation_trajectory: 姿态轨迹
        """
        num_steps = int(duration / dt)
        time_array = np.linspace(0, duration, num_steps)
        
        # 位置线性插值
        position_trajectory = np.zeros((num_steps, 3))
        for i in range(3):
            position_trajectory[:, i] = np.linspace(start_position[i], end_position[i], num_steps)
        
        # 姿态球面线性插值 (SLERP)
        orientation_trajectory = np.zeros((num_steps, 4))
        for i in range(num_steps):
            t = i / (num_steps - 1)
            orientation_trajectory[i] = self._slerp(start_orientation, end_orientation, t)
        
        return time_array, position_trajectory, orientation_trajectory
    
    def _slerp(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        球面线性插值 (SLERP)
        
        Args:
            q1: 起始四元数
            q2: 结束四元数
            t: 插值参数 (0-1)
            
        Returns:
            插值后的四元数
        """
        # 确保四元数为单位四元数
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 计算点积
        dot_product = np.dot(q1, q2)
        
        # 处理四元数方向
        if dot_product < 0:
            q2 = -q2
            dot_product = -dot_product
        
        # 如果四元数接近，使用线性插值
        if dot_product > 0.9995:
            result = q1 + t * (q2 - q1)
        else:
            # 使用SLERP
            theta = np.arccos(dot_product)
            sin_theta = np.sin(theta)
            
            if sin_theta > 1e-6:
                w1 = np.sin((1 - t) * theta) / sin_theta
                w2 = np.sin(t * theta) / sin_theta
                result = w1 * q1 + w2 * q2
            else:
                result = q1
        
        return result / np.linalg.norm(result)
    
    def trajectory_to_joint_angles(self, position_trajectory: np.ndarray,
                                 orientation_trajectory: np.ndarray,
                                 initial_guess: np.ndarray) -> np.ndarray:
        """
        将笛卡尔轨迹转换为关节角度轨迹
        
        Args:
            position_trajectory: 位置轨迹
            orientation_trajectory: 姿态轨迹
            initial_guess: 初始关节角度猜测
            
        Returns:
            关节角度轨迹
        """
        num_steps = len(position_trajectory)
        joint_trajectory = np.zeros((num_steps, len(initial_guess)))
        
        current_guess = initial_guess.copy()
        
        for i in range(num_steps):
            target_position = position_trajectory[i]
            target_orientation = orientation_trajectory[i]
            
            # 求解逆运动学
            joint_angles, success = self.ik_solver.ik_optimization_method(
                target_position, target_orientation, current_guess)
            
            if success:
                joint_trajectory[i] = joint_angles
                current_guess = joint_angles  # 使用当前解作为下次的初始猜测
            else:
                print(f"警告: 第{i}步逆运动学求解失败")
                joint_trajectory[i] = current_guess
        
        return joint_trajectory
    
    def check_trajectory_feasibility(self, trajectory: np.ndarray,
                                   joint_limits: np.ndarray) -> bool:
        """
        检查轨迹可行性
        
        Args:
            trajectory: 关节角度轨迹
            joint_limits: 关节限位
            
        Returns:
            轨迹是否可行
        """
        for i in range(len(trajectory)):
            for j in range(len(trajectory[i])):
                if (trajectory[i, j] < joint_limits[j, 0] or 
                    trajectory[i, j] > joint_limits[j, 1]):
                    return False
        return True
    
    def smooth_trajectory(self, trajectory: np.ndarray, smoothing_factor: float = 0.1) -> np.ndarray:
        """
        平滑轨迹
        
        Args:
            trajectory: 原始轨迹
            smoothing_factor: 平滑因子
            
        Returns:
            平滑后的轨迹
        """
        smoothed = trajectory.copy()
        num_steps, num_joints = trajectory.shape
        
        for i in range(1, num_steps - 1):
            for j in range(num_joints):
                smoothed[i, j] = (trajectory[i, j] + 
                                smoothing_factor * (trajectory[i-1, j] + trajectory[i+1, j])) / (1 + 2 * smoothing_factor)
        
        return smoothed


class PathPlanner:
    """路径规划器"""
    
    def __init__(self, fk_solver: ForwardKinematics, ik_solver: InverseKinematics):
        """
        初始化路径规划器
        
        Args:
            fk_solver: 正向运动学求解器
            ik_solver: 逆向运动学求解器
        """
        self.fk_solver = fk_solver
        self.ik_solver = ik_solver
        self.trajectory_planner = TrajectoryPlanner(fk_solver, ik_solver)
    
    def plan_circular_path(self, center: np.ndarray, radius: float,
                          start_angle: float, end_angle: float,
                          num_points: int = 50) -> Tuple[np.ndarray, np.ndarray]:
        """
        规划圆形路径
        
        Args:
            center: 圆心位置
            radius: 半径
            start_angle: 起始角度
            end_angle: 结束角度
            num_points: 路径点数量
            
        Returns:
            position_path: 位置路径
            orientation_path: 姿态路径
        """
        angles = np.linspace(start_angle, end_angle, num_points)
        
        position_path = np.zeros((num_points, 3))
        orientation_path = np.zeros((num_points, 4))
        
        for i, angle in enumerate(angles):
            # 计算圆上的位置
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
            position_path[i] = [x, y, z]
            
            # 计算朝向圆心的姿态
            # 这里简化处理，实际应用中需要根据具体需求调整
            orientation_path[i] = [1, 0, 0, 0]  # 单位四元数
        
        return position_path, orientation_path
    
    def plan_straight_line_path(self, start_position: np.ndarray, end_position: np.ndarray,
                               start_orientation: np.ndarray, end_orientation: np.ndarray,
                               num_points: int = 50) -> Tuple[np.ndarray, np.ndarray]:
        """
        规划直线路径
        
        Args:
            start_position: 起始位置
            end_position: 结束位置
            start_orientation: 起始姿态
            end_orientation: 结束姿态
            num_points: 路径点数量
            
        Returns:
            position_path: 位置路径
            orientation_path: 姿态路径
        """
        t_values = np.linspace(0, 1, num_points)
        
        position_path = np.zeros((num_points, 3))
        orientation_path = np.zeros((num_points, 4))
        
        for i, t in enumerate(t_values):
            # 位置线性插值
            position_path[i] = (1 - t) * start_position + t * end_position
            
            # 姿态球面线性插值
            orientation_path[i] = self.trajectory_planner._slerp(start_orientation, end_orientation, t)
        
        return position_path, orientation_path 