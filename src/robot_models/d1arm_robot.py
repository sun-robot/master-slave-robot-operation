"""
基于抽象接口的Unitree D1Arm机械臂实现
支持仿真和真实硬件控制
"""

import numpy as np
import time
import json
import threading
import logging
import yaml
from typing import Dict, List, Optional, Tuple, Any
from .robot_interface import RobotInterface, SimulatedRobotInterface, RobotType, EndEffectorType
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics

# 尝试导入Unitree SDK2
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.utils.timerfd import timespec
    from msg.ArmString_ import ArmString_
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False


class D1ArmModel:
    """D1Arm机械臂模型类"""
    
    def __init__(self):
        """初始化D1Arm模型"""
        # D1Arm的DH参数 (a, alpha, d, theta)
        # 基于Unitree D1Arm的规格 - 修正版本
        self.dh_params = np.array([
            [0.0, -np.pi/2, 0.1625, 0.0],      # 关节1
            [0.0, np.pi/2, 0.0, 0.0],          # 关节2
            [0.425, 0.0, 0.0, 0.0],            # 关节3
            [0.0, -np.pi/2, 0.0, 0.0],         # 关节4
            [0.0, np.pi/2, 0.0, 0.0],          # 关节5
            [0.0, 0.0, 0.0, 0.0],              # 关节6
            [0.0, 0.0, 0.1333, 0.0]            # 关节7
        ])
        
        # 关节限位 (弧度)
        self.joint_limits = np.array([
            [-np.pi, np.pi],      # 关节1: ±180°
            [-np.pi/2, np.pi/2],  # 关节2: ±90°
            [-np.pi, np.pi],      # 关节3: ±180°
            [-np.pi/2, np.pi/2],  # 关节4: ±90°
            [-np.pi, np.pi],      # 关节5: ±180°
            [-np.pi/2, np.pi/2],  # 关节6: ±90°
            [-np.pi, np.pi]       # 关节7: ±180°
        ])
        
        # 关节速度限制 (rad/s)
        self.joint_velocity_limits = np.array([
            2.0,  # 关节1
            2.0,  # 关节2
            2.0,  # 关节3
            2.0,  # 关节4
            2.0,  # 关节5
            2.0,  # 关节6
            2.0   # 关节7
        ])
        
        # 关节加速度限制 (rad/s²)
        self.joint_acceleration_limits = np.array([
            5.0,  # 关节1
            5.0,  # 关节2
            5.0,  # 关节3
            5.0,  # 关节4
            5.0,  # 关节5
            5.0,  # 关节6
            5.0   # 关节7
        ])
        
        # 机械臂参数
        self.arm_length = 0.425 + 0.3922 + 0.1333  # 总臂长
        self.payload_capacity = 5.0  # 最大负载 (kg)
        self.repeatability = 0.02    # 重复精度 (mm)
        
        # 工作空间参数
        self.workspace_radius = 0.8  # 工作空间半径 (m)
        self.base_height = 0.1625    # 基座高度 (m)
        
    def get_dh_params(self) -> np.ndarray:
        """获取DH参数"""
        return self.dh_params.copy()
    
    def get_joint_limits(self) -> np.ndarray:
        """获取关节限位"""
        return self.joint_limits.copy()
    
    def get_joint_velocity_limits(self) -> np.ndarray:
        """获取关节速度限制"""
        return self.joint_velocity_limits.copy()
    
    def get_joint_acceleration_limits(self) -> np.ndarray:
        """获取关节加速度限制"""
        return self.joint_acceleration_limits.copy()
    
    def check_joint_limits(self, joint_angles: np.ndarray) -> bool:
        """
        检查关节角度是否在限位范围内
        
        Args:
            joint_angles: 关节角度数组
            
        Returns:
            是否在限位范围内
        """
        if len(joint_angles) != len(self.joint_limits):
            return False
        
        for i, angle in enumerate(joint_angles):
            if angle < self.joint_limits[i, 0] or angle > self.joint_limits[i, 1]:
                return False
        
        return True
    
    def get_workspace_center(self) -> np.ndarray:
        """获取工作空间中心位置"""
        return np.array([0.0, 0.0, self.base_height + self.arm_length / 2])
    
    def get_robot_info(self) -> Dict[str, Any]:
        """获取机械臂信息"""
        return {
            'name': 'D1Arm',
            'manufacturer': 'Unitree',
            'dof': len(self.dh_params),
            'arm_length': self.arm_length,
            'payload_capacity': self.payload_capacity,
            'repeatability': self.repeatability,
            'workspace_radius': self.workspace_radius,
            'base_height': self.base_height
        }
    
    def save_config(self, filepath: str):
        """保存配置到文件"""
        config = {
            'dh_params': self.dh_params.tolist(),
            'joint_limits': self.joint_limits.tolist(),
            'joint_velocity_limits': self.joint_velocity_limits.tolist(),
            'joint_acceleration_limits': self.joint_acceleration_limits.tolist(),
            'robot_info': self.get_robot_info()
        }
        
        with open(filepath, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
    
    @classmethod
    def load_config(cls, filepath: str) -> 'D1ArmModel':
        """从文件加载配置"""
        with open(filepath, 'r') as f:
            config = yaml.safe_load(f)
        
        model = cls()
        model.dh_params = np.array(config['dh_params'])
        model.joint_limits = np.array(config['joint_limits'])
        model.joint_velocity_limits = np.array(config['joint_velocity_limits'])
        model.joint_acceleration_limits = np.array(config['joint_acceleration_limits'])
        
        return model


class D1ArmSimulator:
    """D1Arm仿真器"""
    
    def __init__(self, model: D1ArmModel):
        """
        初始化仿真器
        
        Args:
            model: D1Arm模型
        """
        self.model = model
        self.current_joint_angles = np.zeros(model.dh_params.shape[0])
        self.current_joint_velocities = np.zeros(model.dh_params.shape[0])
        
    def set_joint_angles(self, joint_angles: np.ndarray):
        """设置关节角度"""
        if self.model.check_joint_limits(joint_angles):
            self.current_joint_angles = joint_angles.copy()
        else:
            raise ValueError("关节角度超出限位范围")
    
    def get_current_joint_angles(self) -> np.ndarray:
        """获取当前关节角度"""
        return self.current_joint_angles.copy()
    
    def get_current_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        return self.current_joint_velocities.copy()
    
    def simulate_motion(self, target_joint_angles: np.ndarray, dt: float = 0.01):
        """
        仿真运动过程
        
        Args:
            target_joint_angles: 目标关节角度
            dt: 时间步长
        """
        # 简单的线性插值仿真
        max_velocity = np.min(self.model.joint_velocity_limits)
        angle_diff = target_joint_angles - self.current_joint_angles
        
        # 计算运动时间
        max_angle_diff = np.max(np.abs(angle_diff))
        if max_angle_diff > 0:
            motion_time = max_angle_diff / max_velocity
            steps = int(motion_time / dt)
            
            for i in range(steps):
                # 线性插值
                t = i / steps
                self.current_joint_angles = (1 - t) * self.current_joint_angles + t * target_joint_angles
                
                # 计算关节速度
                self.current_joint_velocities = angle_diff / motion_time
                
                # 这里可以添加更多的仿真逻辑，如碰撞检测等
                
        else:
            self.current_joint_velocities = np.zeros_like(self.current_joint_angles)


class D1ArmSimRobot(SimulatedRobotInterface):
    """
    Unitree D1Arm机械臂实现
    基于抽象接口，支持仿真和真实硬件
    """
    
    def __init__(self, robot_id: str = "d1arm_0", 
                 use_pinocchio: bool = False,
                 urdf_path: Optional[str] = None):
        """
        初始化D1Arm机械臂
        
        Args:
            robot_id: 机械臂唯一标识符
            use_pinocchio: 是否使用Pinocchio IK求解器
            urdf_path: URDF文件路径（Pinocchio使用）
        """
        super().__init__(RobotType.UNITREE_D1, robot_id)
        
        # 加载D1Arm模型
        self._model = D1ArmModel()
        
        # 初始化运动学求解器
        self._fk_solver = ForwardKinematics(self._model.get_dh_params())
        
        # 初始化逆运动学求解器
        ik_kwargs = {
            'fk_solver': self._fk_solver,
            'joint_limits': self._model.get_joint_limits()
        }
        
        if use_pinocchio and urdf_path:
            ik_kwargs.update({
                'urdf_path': urdf_path,
                'urdf_root_path': "assets/d1arm/",
                'end_effector_joint': "joint_7",
                'end_effector_link': "link_7",
                'end_effector_offset': np.array([0.05, 0.0, 0.0]),
                'use_pinocchio': True
            })
        
        self._ik_solver = InverseKinematics(**ik_kwargs)
        
        # 设置初始关节位置为零位
        self._joint_positions = np.zeros(self.dof)
        self._joint_velocities = np.zeros(self.dof)
        self._joint_torques = np.zeros(self.dof)
        
        # 工具变换矩阵
        self._tool_transform = np.eye(4)
        self._base_transform = np.eye(4)
        
    # ===================== 基本属性实现 =====================
    
    @property
    def name(self) -> str:
        """机械臂名称"""
        return "Unitree D1Arm"
    
    @property
    def dof(self) -> int:
        """自由度数量"""
        return 7
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        return [f"joint_{i+1}" for i in range(7)]
    
    @property
    def joint_limits(self) -> np.ndarray:
        """关节限位"""
        return self._model.get_joint_limits()
    
    @property
    def velocity_limits(self) -> np.ndarray:
        """关节速度限位"""
        return self._model.get_joint_velocity_limits()
    
    @property
    def acceleration_limits(self) -> np.ndarray:
        """关节加速度限位"""
        return self._model.get_joint_acceleration_limits()
    
    # ===================== DH参数和运动学实现 =====================
    
    def get_dh_params(self) -> np.ndarray:
        """获取DH参数"""
        return self._model.get_dh_params()
    
    def get_base_transform(self) -> np.ndarray:
        """获取基座变换矩阵"""
        return self._base_transform.copy()
    
    def get_end_effector_transform(self) -> np.ndarray:
        """获取末端执行器变换"""
        return self._tool_transform.copy()
    
    # ===================== 状态获取实现 =====================
    
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位姿"""
        position, orientation = self._fk_solver.forward_kinematics(self._joint_positions)
        
        # 应用工具变换
        if not np.allclose(self._tool_transform, np.eye(4)):
            # 构建当前末端变换矩阵
            current_transform = np.eye(4)
            current_transform[:3, 3] = position
            current_transform[:3, :3] = self._quaternion_to_rotation_matrix(orientation)
            
            # 应用工具变换
            final_transform = current_transform @ self._tool_transform
            
            position = final_transform[:3, 3]
            orientation = self._rotation_matrix_to_quaternion(final_transform[:3, :3])
        
        return position, orientation
    
    # ===================== 运动控制实现 =====================
    
    def move_to_pose(self, position: np.ndarray, orientation: np.ndarray,
                    velocity_scale: float = 1.0,
                    acceleration_scale: float = 1.0,
                    blocking: bool = True) -> bool:
        """移动到指定末端位姿"""
        if not self.is_ready():
            return False
        
        # 使用逆运动学求解关节角度
        initial_guess = self._joint_positions.copy()
        
        # 选择最佳的IK方法
        if hasattr(self._ik_solver, 'use_pinocchio') and self._ik_solver.use_pinocchio:
            joint_angles, success = self._ik_solver.ik_pinocchio_method(
                position, orientation, initial_guess
            )
        else:
            joint_angles, success = self._ik_solver.ik_optimization_method(
                position, orientation, initial_guess
            )
        
        if not success:
            self._set_state(self.state.__class__.ERROR, "逆运动学求解失败")
            return False
        
        # 移动到计算出的关节位置
        return self.move_to_joint_positions(joint_angles, velocity_scale, acceleration_scale, blocking)
    
    def move_linear(self, target_position: np.ndarray, 
                   target_orientation: np.ndarray,
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   blocking: bool = True) -> bool:
        """直线运动到目标位姿"""
        if not self.is_ready():
            return False
        
        # 获取当前位姿
        current_position, current_orientation = self.get_end_effector_pose()
        
        # 计算轨迹点数量
        distance = np.linalg.norm(target_position - current_position)
        num_points = max(10, int(distance / 0.01))  # 每厘米一个点
        
        # 生成线性轨迹
        for i in range(num_points + 1):
            t = i / num_points
            
            # 位置插值
            interp_position = current_position + t * (target_position - current_position)
            
            # 姿态插值（简单的球面线性插值）
            interp_orientation = self._slerp_quaternion(current_orientation, target_orientation, t)
            
            # 移动到插值点
            if not self.move_to_pose(interp_position, interp_orientation, 
                                   velocity_scale=velocity, blocking=True):
                return False
        
        return True
    
    # ===================== 高级功能实现 =====================
    
    def get_jacobian(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """获取雅可比矩阵"""
        if joint_positions is None:
            joint_positions = self._joint_positions
        
        return self._fk_solver.jacobian(joint_positions)
    
    def solve_ik(self, target_pose: np.ndarray, current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """求解逆运动学"""
        # 从4x4位姿矩阵提取位置和姿态
        position = target_pose[:3, 3]
        orientation = self._rotation_matrix_to_quaternion(target_pose[:3, :3])
        
        # 如果没有提供当前关节角度，使用当前位置
        if current_joint_angles is None:
            current_joint_angles = self.get_joint_positions()
        
        # 选择最佳的IK方法
        if hasattr(self._ik_solver, 'use_pinocchio') and self._ik_solver.use_pinocchio:
            return self._ik_solver.ik_pinocchio_method(position, orientation, current_joint_angles)
        else:
            return self._ik_solver.ik_optimization_method(position, orientation, current_joint_angles)
    
    # ===================== 末端执行器控制 =====================
    
    def get_end_effector_type(self) -> EndEffectorType:
        """获取末端执行器类型"""
        return EndEffectorType.GRIPPER  # D1Arm通常配置夹爪
    
    def open_gripper(self, width: float = 1.0) -> bool:
        """打开夹爪"""
        # 仿真实现 - 真实硬件需要具体的驱动代码
        print(f"D1Arm夹爪打开到 {width*100:.1f}%")
        return True
    
    def close_gripper(self, force: float = 1.0) -> bool:
        """关闭夹爪"""
        # 仿真实现 - 真实硬件需要具体的驱动代码
        print(f"D1Arm夹爪以 {force*100:.1f}% 力度关闭")
        return True
    
    def get_gripper_state(self) -> Dict:
        """获取夹爪状态"""
        return {
            "type": "parallel_gripper",
            "available": True,
            "width": 0.08,  # 假设的夹爪宽度
            "force": 0.0,
            "is_grasping": False
        }
    
    # ===================== 工具和坐标系 =====================
    
    def set_tool_transform(self, transform: np.ndarray) -> bool:
        """设置工具坐标系变换"""
        if transform.shape != (4, 4):
            return False
        
        self._tool_transform = transform.copy()
        return True
    
    def set_base_transform(self, transform: np.ndarray) -> bool:
        """设置基座坐标系变换"""
        if transform.shape != (4, 4):
            return False
        
        self._base_transform = transform.copy()
        return True
    
    # ===================== 内部工具方法 =====================
    
    def _quaternion_to_rotation_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵"""
        w, x, y, z = quaternion
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
    
    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        trace = np.trace(rotation_matrix)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                x = 0.25 * s
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                y = 0.25 * s
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                z = 0.25 * s
        
        return np.array([w, x, y, z])
    
    def _slerp_quaternion(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """四元数球面线性插值"""
        dot = np.dot(q1, q2)
        
        # 如果点积为负，反转一个四元数以选择较短路径
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # 如果四元数很接近，使用线性插值
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # 计算角度
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2


class D1ArmHardware(D1ArmSimRobot):
    """
    Unitree D1Arm真实硬件接口
    继承自D1ArmRobot，添加硬件通信功能
    """
    
    def __init__(self, robot_id: str = "d1arm_hardware",
                 connection_config: Optional[Dict] = None,
                 arm_id: int = 1):
        """
        初始化D1Arm硬件接口
        
        Args:
            robot_id: 机械臂标识符
            connection_config: 连接配置
            arm_id: 机械臂ID (用于SDK通信)
        """
        super().__init__(robot_id)
        self._connection_config = connection_config or {}
        self._hardware_connected = False
        self.arm_id = arm_id
        self._sdk_available = SDK_AVAILABLE
        
        # SDK通信组件
        self._subscriber = None
        self._publisher = None
        self._feedback_thread = None
        self._stop_feedback = False
        
        # 状态变量
        self._last_feedback_time = 0
        self._command_seq = 0
        
        # 状态锁
        self._state_lock = threading.Lock()
        
        # 日志
        self.logger = logging.getLogger(__name__)
        
        if not self._sdk_available:
            self.logger.warning("Unitree SDK2未安装，将使用仿真模式")
        
    def connect(self) -> bool:
        """连接硬件"""
        if not self._sdk_available:
            self.logger.error("Unitree SDK2未安装，无法连接硬件")
            return False
        
        try:
            self.logger.info(f"正在连接D1Arm硬件 (ID: {self.arm_id})...")
            
            # 初始化通道工厂
            ChannelFactoryInitialize()
            
            # 创建订阅者 (反馈信息)
            feedback_topic = f"rt/arm_Feedback_{self.arm_id}"
            self._subscriber = ChannelSubscriber(feedback_topic, ArmString_)
            self._subscriber.Init()
            
            # 创建发布者 (控制命令)
            command_topic = "rt/arm_Command"
            self._publisher = ChannelPublisher(command_topic, ArmString_)
            self._publisher.Init()
            
            # 启动反馈监听线程
            self._start_feedback_thread()
            
            # 等待初始反馈
            start_time = time.time()
            while time.time() - start_time < 5.0:  # 等待5秒
                if self._last_feedback_time > start_time:
                    break
                time.sleep(0.1)
            
            if self._last_feedback_time > start_time:
                self._hardware_connected = True
                self._set_state(self.state.__class__.IDLE)
                self.logger.info("D1Arm硬件连接成功")
                return True
            else:
                self.logger.error("未收到机械臂反馈，连接失败")
                self.disconnect()
                return False
                
        except Exception as e:
            self.logger.error(f"连接D1Arm硬件失败: {e}")
            self._set_state(self.state.__class__.ERROR, f"硬件连接失败: {str(e)}")
            self.disconnect()
            return False
    
    def disconnect(self) -> bool:
        """断开硬件连接"""
        try:
            self.logger.info("正在断开D1Arm硬件连接...")
            
            # 停止反馈线程
            self._stop_feedback_thread()
            
            # 关闭通道
            if self._subscriber:
                self._subscriber.Close()
                self._subscriber = None
            
            if self._publisher:
                # 发送停止命令
                try:
                    self._send_stop_command()
                except:
                    pass
                self._publisher = None
            
            self._hardware_connected = False
            self._set_state(self.state.__class__.DISABLED)
            self.logger.info("D1Arm硬件已断开连接")
            return True
            
        except Exception as e:
            self.logger.error(f"断开连接时发生错误: {e}")
            return False
    
    def _start_feedback_thread(self):
        """启动反馈监听线程"""
        self._stop_feedback = False
        self._feedback_thread = threading.Thread(target=self._feedback_loop)
        self._feedback_thread.daemon = True
        self._feedback_thread.start()
    
    def _stop_feedback_thread(self):
        """停止反馈监听线程"""
        self._stop_feedback = True
        if self._feedback_thread and self._feedback_thread.is_alive():
            self._feedback_thread.join(timeout=2.0)
    
    def _feedback_loop(self):
        """反馈监听循环"""
        self.logger.info("反馈监听线程启动")
        
        while not self._stop_feedback and self._subscriber:
            try:
                # 读取反馈消息，参考用户提供的代码
                start_time = time.time()
                msg = self._subscriber.Read(10)  # 10ms超时
                end_time = time.time()
                
                if msg is not None:
                    self._process_feedback_message(msg)
                    
            except Exception as e:
                self.logger.error(f"反馈处理错误: {e}")
                time.sleep(0.01)
        
        self.logger.info("反馈监听线程结束")
    
    def _process_feedback_message(self, msg):
        """处理反馈消息"""
        try:
            # 解析JSON数据，参考用户提供的代码
            msg_data = json.loads(msg.string_data)
            
            # 检查是否为反馈消息 (funcode=1)
            if msg_data.get('funcode') == 1:
                data = msg_data.get('data', {})
                
                # 更新关节状态
                with self._state_lock:
                    # 提取关节角度 (j1, j2, ..., j7)
                    joint_angles = []
                    for i in range(7):
                        joint_key = f"j{i+1}"  # j1, j2, ..., j7
                        if joint_key in data:
                            joint_angles.append(data[joint_key])
                    
                    if len(joint_angles) == 7:
                        self._joint_positions = np.array(joint_angles)
                    
                    # 更新时间戳
                    self._last_feedback_time = time.time()
                
                # 状态检查
                if self.state == self.state.__class__.DISABLED:
                    self._set_state(self.state.__class__.IDLE)
                    
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON解析错误: {e}")
        except Exception as e:
            self.logger.error(f"反馈消息处理错误: {e}")
    
    def get_joint_positions(self) -> np.ndarray:
        """从硬件获取关节位置"""
        if not self._hardware_connected:
            return super().get_joint_positions()
        
        # 从SDK反馈获取关节位置
        with self._state_lock:
            return self._joint_positions.copy()
    
    def move_to_joint_positions(self, positions: np.ndarray, 
                               velocity_scale: float = 1.0,
                               acceleration_scale: float = 1.0,
                               blocking: bool = True) -> bool:
        """控制硬件移动到指定关节位置"""
        if not self._hardware_connected:
            return super().move_to_joint_positions(positions, velocity_scale, acceleration_scale, blocking)
        
        if not self._validate_joint_positions(positions):
            return False
        
        try:
            # 通过SDK发送关节位置控制命令
            success = self._send_joint_command(positions, velocity_scale)
            
            if success and blocking:
                # 等待运动完成
                return self._wait_for_motion_complete(positions)
            
            return success
            
        except Exception as e:
            self.logger.error(f"关节运动失败: {e}")
            self._set_state(self.state.__class__.ERROR, str(e))
            return False
    
    def _send_joint_command(self, joint_positions: np.ndarray, velocity_scale: float = 1.0) -> bool:
        """发送关节位置控制命令"""
        if not self._publisher:
            return False
        
        try:
            # 构建命令消息，参考用户提供的代码格式
            self._command_seq += 1
            
            command_data = {
                "seq": self._command_seq,
                "address": self.arm_id,
                "funcode": 2,  # 控制命令
                "data": {
                    "mode": 0,  # 位置控制模式
                    "j1": float(joint_positions[0]),
                    "j2": float(joint_positions[1]),
                    "j3": float(joint_positions[2]),
                    "j4": float(joint_positions[3]),
                    "j5": float(joint_positions[4]),
                    "j6": float(joint_positions[5]),
                    "j7": float(joint_positions[6]),
                    "velocity_scale": float(velocity_scale)
                }
            }
            
            # 转换为JSON字符串
            json_command = json.dumps(command_data, indent=4)
            
            # 发布命令
            self._publisher.Write(ArmString_(json_command))
            
            self.logger.debug(f"发送关节命令: {joint_positions}")
            return True
            
        except Exception as e:
            self.logger.error(f"发送关节命令失败: {e}")
            return False
    
    def _send_stop_command(self, emergency: bool = False) -> bool:
        """发送停止命令"""
        if not self._publisher:
            return False
        
        try:
            self._command_seq += 1
            
            stop_data = {
                "seq": self._command_seq,
                "address": self.arm_id,
                "funcode": 2,
                "data": {
                    "mode": 99 if emergency else 10,  # 99=紧急停止, 10=正常停止
                }
            }
            
            json_command = json.dumps(stop_data, indent=4)
            self._publisher.Write(ArmString_(json_command))
            
            self.logger.info(f"发送停止命令 (紧急: {emergency})")
            return True
            
        except Exception as e:
            self.logger.error(f"发送停止命令失败: {e}")
            return False
    
    def _wait_for_motion_complete(self, target_positions: np.ndarray, 
                                 timeout: float = 30.0) -> bool:
        """等待运动完成"""
        start_time = time.time()
        position_tolerance = 0.02  # 约1度容差
        
        while time.time() - start_time < timeout:
            current_positions = self.get_joint_positions()
            
            # 检查是否到达目标位置
            position_error = np.abs(current_positions - target_positions)
            if np.all(position_error < position_tolerance):
                self.logger.debug("运动完成")
                return True
            
            time.sleep(0.1)
        
        self.logger.warning("运动超时")
        return False
    
    def is_ready(self) -> bool:
        """检查硬件是否就绪"""
        return (self._hardware_connected and 
                super().is_ready() and 
                self._publisher is not None and 
                time.time() - self._last_feedback_time < 2.0)  # 2秒内有反馈
    
    def stop_motion(self, emergency: bool = False) -> bool:
        """停止运动"""
        if not self._hardware_connected:
            return super().stop_motion(emergency)
        
        try:
            success = self._send_stop_command(emergency)
            if emergency:
                self._set_state(self.state.__class__.EMERGENCY_STOP)
            return success
        except Exception as e:
            self.logger.error(f"停止运动失败: {e}")
            return False 