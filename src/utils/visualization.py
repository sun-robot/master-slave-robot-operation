"""
可视化模块
用于显示机械臂的运动轨迹和状态
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple, Optional
import transforms3d as t3d
from ..kinematics.forward_kinematics import ForwardKinematics
from ..robot_models.d1arm_robot import D1ArmModel


class RobotVisualizer:
    """机械臂可视化器"""
    
    def __init__(self, robot_model: D1ArmModel, fk_solver: ForwardKinematics):
        """
        初始化可视化器
        
        Args:
            robot_model: 机械臂模型
            fk_solver: 正向运动学求解器
        """
        self.robot_model = robot_model
        self.fk_solver = fk_solver
        self.fig = None
        self.ax = None
        
    def setup_plot(self, figsize: Tuple[int, int] = (10, 8)):
        """设置绘图环境"""
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('机械臂可视化')
        
    def plot_robot_configuration(self, joint_angles: np.ndarray, 
                               color: str = 'blue', alpha: float = 0.7):
        """
        绘制机械臂配置
        
        Args:
            joint_angles: 关节角度
            color: 颜色
            alpha: 透明度
        """
        if self.ax is None:
            self.setup_plot()
        
        # 获取所有关节位置
        joint_positions = self.fk_solver.get_joint_positions(joint_angles)
        
        # 绘制连杆
        for i in range(len(joint_positions) - 1):
            start_pos = joint_positions[i]
            end_pos = joint_positions[i + 1]
            
            self.ax.plot([start_pos[0], end_pos[0]], 
                        [start_pos[1], end_pos[1]], 
                        [start_pos[2], end_pos[2]], 
                        color=color, linewidth=3, alpha=alpha)
        
        # 绘制关节
        for i, pos in enumerate(joint_positions):
            self.ax.scatter(pos[0], pos[1], pos[2], 
                          color='red', s=50, alpha=alpha)
            
            # 添加关节标签
            self.ax.text(pos[0], pos[1], pos[2], f'J{i+1}', 
                        fontsize=8, ha='center', va='bottom')
        
        # 绘制末端执行器
        end_pos = joint_positions[-1]
        self.ax.scatter(end_pos[0], end_pos[1], end_pos[2], 
                       color='green', s=100, alpha=alpha, marker='*')
        self.ax.text(end_pos[0], end_pos[1], end_pos[2], 'EE', 
                    fontsize=10, ha='center', va='bottom')
    
    def plot_workspace(self, center: np.ndarray, radius: float, 
                      color: str = 'gray', alpha: float = 0.1):
        """
        绘制工作空间
        
        Args:
            center: 工作空间中心
            radius: 工作空间半径
            color: 颜色
            alpha: 透明度
        """
        if self.ax is None:
            self.setup_plot()
        
        # 绘制球形工作空间
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        self.ax.plot_surface(x, y, z, color=color, alpha=alpha)
    
    def plot_trajectory(self, trajectory: np.ndarray, 
                       color: str = 'orange', linewidth: int = 2):
        """
        绘制轨迹
        
        Args:
            trajectory: 轨迹点数组，每行包含 [x, y, z]
            color: 颜色
            linewidth: 线宽
        """
        if self.ax is None:
            self.setup_plot()
        
        if len(trajectory) > 1:
            self.ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                        color=color, linewidth=linewidth, alpha=0.8)
    
    def plot_target_pose(self, position: np.ndarray, orientation: np.ndarray,
                        size: float = 0.1, color: str = 'red'):
        """
        绘制目标位姿
        
        Args:
            position: 目标位置
            orientation: 目标姿态
            size: 坐标轴大小
            color: 颜色
        """
        if self.ax is None:
            self.setup_plot()
        
        # 将四元数转换为旋转矩阵
        rotation_matrix = t3d.quaternions.quat2mat(orientation)
        
        # 绘制坐标轴
        origin = position
        x_axis = origin + size * rotation_matrix[:, 0]
        y_axis = origin + size * rotation_matrix[:, 1]
        z_axis = origin + size * rotation_matrix[:, 2]
        
        self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 
                    color='red', linewidth=2)
        self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 
                    color='green', linewidth=2)
        self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 
                    color='blue', linewidth=2)
        
        # 添加标签
        self.ax.text(x_axis[0], x_axis[1], x_axis[2], 'X', fontsize=8)
        self.ax.text(y_axis[0], y_axis[1], y_axis[2], 'Y', fontsize=8)
        self.ax.text(z_axis[0], z_axis[1], z_axis[2], 'Z', fontsize=8)
    
    def animate_trajectory(self, joint_trajectory: np.ndarray, 
                          interval: int = 100, repeat: bool = True):
        """
        动画显示轨迹
        
        Args:
            joint_trajectory: 关节角度轨迹
            interval: 动画间隔（毫秒）
            repeat: 是否重复播放
        """
        if self.ax is None:
            self.setup_plot()
        
        def animate(frame):
            self.ax.clear()
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_zlabel('Z (m)')
            self.ax.set_title(f'机械臂轨迹动画 (帧 {frame+1}/{len(joint_trajectory)})')
            
            # 绘制当前配置
            self.plot_robot_configuration(joint_trajectory[frame])
            
            # 设置坐标轴范围
            self._set_axis_limits()
        
        anim = FuncAnimation(self.fig, animate, frames=len(joint_trajectory),
                           interval=interval, repeat=repeat)
        return anim
    
    def _set_axis_limits(self):
        """设置坐标轴范围"""
        # 获取当前图形的边界
        x_limits = self.ax.get_xlim()
        y_limits = self.ax.get_ylim()
        z_limits = self.ax.get_zlim()
        
        # 设置统一的边界
        max_range = max(x_limits[1] - x_limits[0],
                       y_limits[1] - y_limits[0],
                       z_limits[1] - z_limits[0])
        
        center_x = (x_limits[0] + x_limits[1]) / 2
        center_y = (y_limits[0] + y_limits[1]) / 2
        center_z = (z_limits[0] + z_limits[1]) / 2
        
        self.ax.set_xlim(center_x - max_range/2, center_x + max_range/2)
        self.ax.set_ylim(center_y - max_range/2, center_y + max_range/2)
        self.ax.set_zlim(center_z - max_range/2, center_z + max_range/2)
    
    def show(self):
        """显示图形"""
        if self.fig is not None:
            plt.show()
    
    def save_animation(self, anim, filename: str, fps: int = 10):
        """
        保存动画
        
        Args:
            anim: 动画对象
            filename: 文件名
            fps: 帧率
        """
        try:
            anim.save(filename, writer='pillow', fps=fps)
            print(f"动画已保存到: {filename}")
        except Exception as e:
            print(f"保存动画失败: {e}")


class TrajectoryAnalyzer:
    """轨迹分析器"""
    
    def __init__(self):
        """初始化轨迹分析器"""
        pass
    
    def plot_joint_angles(self, time_array: np.ndarray, joint_trajectory: np.ndarray,
                         joint_names: Optional[List[str]] = None):
        """
        绘制关节角度轨迹
        
        Args:
            time_array: 时间数组
            joint_trajectory: 关节角度轨迹
            joint_names: 关节名称列表
        """
        num_joints = joint_trajectory.shape[1]
        
        if joint_names is None:
            joint_names = [f'关节{i+1}' for i in range(num_joints)]
        
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2*num_joints))
        if num_joints == 1:
            axes = [axes]
        
        for i in range(num_joints):
            axes[i].plot(time_array, joint_trajectory[:, i], 'b-', linewidth=2)
            axes[i].set_ylabel(f'{joint_names[i]} (rad)')
            axes[i].grid(True)
            
            if i == num_joints - 1:
                axes[i].set_xlabel('时间 (s)')
        
        plt.tight_layout()
        plt.show()
    
    def plot_joint_velocities(self, time_array: np.ndarray, velocity_trajectory: np.ndarray,
                             joint_names: Optional[List[str]] = None):
        """
        绘制关节速度轨迹
        
        Args:
            time_array: 时间数组
            velocity_trajectory: 关节速度轨迹
            joint_names: 关节名称列表
        """
        num_joints = velocity_trajectory.shape[1]
        
        if joint_names is None:
            joint_names = [f'关节{i+1}' for i in range(num_joints)]
        
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2*num_joints))
        if num_joints == 1:
            axes = [axes]
        
        for i in range(num_joints):
            axes[i].plot(time_array, velocity_trajectory[:, i], 'r-', linewidth=2)
            axes[i].set_ylabel(f'{joint_names[i]} 速度 (rad/s)')
            axes[i].grid(True)
            
            if i == num_joints - 1:
                axes[i].set_xlabel('时间 (s)')
        
        plt.tight_layout()
        plt.show()
    
    def plot_cartesian_trajectory(self, time_array: np.ndarray, 
                                 position_trajectory: np.ndarray,
                                 orientation_trajectory: np.ndarray):
        """
        绘制笛卡尔空间轨迹
        
        Args:
            time_array: 时间数组
            position_trajectory: 位置轨迹
            orientation_trajectory: 姿态轨迹
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # 绘制位置轨迹
        axes[0].plot(time_array, position_trajectory[:, 0], 'r-', label='X', linewidth=2)
        axes[0].plot(time_array, position_trajectory[:, 1], 'g-', label='Y', linewidth=2)
        axes[0].plot(time_array, position_trajectory[:, 2], 'b-', label='Z', linewidth=2)
        axes[0].set_ylabel('位置 (m)')
        axes[0].legend()
        axes[0].grid(True)
        
        # 绘制姿态轨迹（欧拉角）
        euler_trajectory = np.zeros((len(time_array), 3))
        for i in range(len(time_array)):
            euler_trajectory[i] = t3d.euler.quat2euler(orientation_trajectory[i])
        
        axes[1].plot(time_array, euler_trajectory[:, 0], 'r-', label='Roll', linewidth=2)
        axes[1].plot(time_array, euler_trajectory[:, 1], 'g-', label='Pitch', linewidth=2)
        axes[1].plot(time_array, euler_trajectory[:, 2], 'b-', label='Yaw', linewidth=2)
        axes[1].set_ylabel('姿态 (rad)')
        axes[1].set_xlabel('时间 (s)')
        axes[1].legend()
        axes[1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def analyze_trajectory_smoothness(self, joint_trajectory: np.ndarray, 
                                    time_array: np.ndarray) -> dict:
        """
        分析轨迹平滑性
        
        Args:
            joint_trajectory: 关节角度轨迹
            time_array: 时间数组
            
        Returns:
            分析结果字典
        """
        # 计算速度
        dt = time_array[1] - time_array[0]
        velocity = np.gradient(joint_trajectory, dt, axis=0)
        
        # 计算加速度
        acceleration = np.gradient(velocity, dt, axis=0)
        
        # 计算加加速度（jerk）
        jerk = np.gradient(acceleration, dt, axis=0)
        
        # 计算平滑性指标
        smoothness_metrics = {
            'max_velocity': np.max(np.abs(velocity), axis=0),
            'max_acceleration': np.max(np.abs(acceleration), axis=0),
            'max_jerk': np.max(np.abs(jerk), axis=0),
            'rms_velocity': np.sqrt(np.mean(velocity**2, axis=0)),
            'rms_acceleration': np.sqrt(np.mean(acceleration**2, axis=0)),
            'rms_jerk': np.sqrt(np.mean(jerk**2, axis=0))
        }
        
        return smoothness_metrics
    
    def print_trajectory_summary(self, joint_trajectory: np.ndarray, 
                               time_array: np.ndarray):
        """
        打印轨迹摘要
        
        Args:
            joint_trajectory: 关节角度轨迹
            time_array: 时间数组
        """
        duration = time_array[-1] - time_array[0]
        num_steps = len(time_array)
        
        print("=== 轨迹摘要 ===")
        print(f"总时间: {duration:.2f} 秒")
        print(f"步数: {num_steps}")
        print(f"时间步长: {time_array[1] - time_array[0]:.3f} 秒")
        print(f"控制频率: {1/(time_array[1] - time_array[0]):.1f} Hz")
        
        # 关节角度范围
        print("\n关节角度范围:")
        for i in range(joint_trajectory.shape[1]):
            min_angle = np.min(joint_trajectory[:, i])
            max_angle = np.max(joint_trajectory[:, i])
            range_angle = max_angle - min_angle
            print(f"  关节{i+1}: [{min_angle:.3f}, {max_angle:.3f}] (范围: {range_angle:.3f} rad)")
        
        # 平滑性分析
        smoothness = self.analyze_trajectory_smoothness(joint_trajectory, time_array)
        print("\n平滑性指标:")
        for i in range(joint_trajectory.shape[1]):
            print(f"  关节{i+1}:")
            print(f"    最大速度: {smoothness['max_velocity'][i]:.3f} rad/s")
            print(f"    最大加速度: {smoothness['max_acceleration'][i]:.3f} rad/s²")
            print(f"    最大加加速度: {smoothness['max_jerk'][i]:.3f} rad/s³") 