"""
Franka Emika Panda 机械臂实现
支持Franka Panda机械臂
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from .robot_interface import RobotInterface, SimulatedRobotInterface, RobotType, EndEffectorType
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics


class FrankaModel:
    """Franka Panda机械臂模型"""
    
    def __init__(self):
        self._setup_parameters()
    
    def _setup_parameters(self):
        """设置Franka Panda参数"""
        # Franka Panda的DH参数 (a, alpha, d, theta)
        self.dh_params = np.array([
            [0.0, 0.0, 0.333, 0.0],          # 关节1
            [0.0, -np.pi/2, 0.0, 0.0],       # 关节2
            [0.0, np.pi/2, 0.316, 0.0],      # 关节3
            [0.0825, np.pi/2, 0.0, 0.0],     # 关节4
            [-0.0825, -np.pi/2, 0.384, 0.0], # 关节5
            [0.0, np.pi/2, 0.0, 0.0],        # 关节6
            [0.088, np.pi/2, 0.107, 0.0]     # 关节7
        ])
        
        # 关节限位 (弧度)
        self.joint_limits = np.array([
            [-2.8973, 2.8973],      # 关节1: ±166°
            [-1.7628, 1.7628],      # 关节2: ±101°
            [-2.8973, 2.8973],      # 关节3: ±166°
            [-3.0718, -0.0698],     # 关节4: -176° to -4°
            [-2.8973, 2.8973],      # 关节5: ±166°
            [-0.0175, 3.7525],      # 关节6: -1° to +215°
            [-2.8973, 2.8973]       # 关节7: ±166°
        ])
        
        # 关节速度限制 (rad/s)
        self.joint_velocity_limits = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        
        # 关节加速度限制 (rad/s²)
        self.joint_acceleration_limits = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
        
        # 关节力矩限制 (Nm)
        self.joint_torque_limits = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
        
        # 机械臂参数
        self.payload_capacity = 3.0  # kg
        self.reach = 0.855  # m
        self.repeatability = 0.1  # mm
    
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
    
    def get_joint_torque_limits(self) -> np.ndarray:
        """获取关节力矩限制"""
        return self.joint_torque_limits.copy()


class FrankaSimRobot(SimulatedRobotInterface):
    """
    Franka Emika Panda机械臂实现
    基于抽象接口，支持仿真和真实硬件
    """
    
    def __init__(self, robot_id: str = "franka_panda",
                 use_pinocchio: bool = False, urdf_path: Optional[str] = None):
        """
        初始化Franka Panda机械臂
        
        Args:
            robot_id: 机械臂唯一标识符
            use_pinocchio: 是否使用Pinocchio IK求解器
            urdf_path: URDF文件路径（Pinocchio使用）
        """
        super().__init__(RobotType.FRANKA_PANDA, robot_id)
        
        # 加载Franka模型
        self._model = FrankaModel()
        
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
                'urdf_root_path': "assets/franka/",
                'end_effector_joint': "panda_joint7",
                'end_effector_link': "panda_link7",
                'end_effector_offset': np.array([0.0, 0.0, 0.107]),
                'use_pinocchio': True
            })
        
        self._ik_solver = InverseKinematics(**ik_kwargs)
        
        # 设置初始关节位置
        self._joint_positions = np.zeros(self.dof)
        self._joint_velocities = np.zeros(self.dof)
        self._joint_torques = np.zeros(self.dof)
        
        # 工具变换矩阵
        self._tool_transform = np.eye(4)
        self._base_transform = np.eye(4)
        
        # Franka特有参数
        self._gripper_width = 0.08  # 夹爪宽度
        self._gripper_grasping = False
        self._collision_detection = True
        self._impedance_control = False
    
    # ===================== 基本属性实现 =====================
    
    @property
    def name(self) -> str:
        """机械臂名称"""
        return "Franka Emika Panda"
    
    @property
    def dof(self) -> int:
        """自由度数量"""
        return 7
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        return [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        ]
    
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
    
    @property
    def torque_limits(self) -> np.ndarray:
        """关节力矩限位"""
        return self._model.get_joint_torque_limits()
    
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
            current_transform = np.eye(4)
            current_transform[:3, 3] = position
            current_transform[:3, :3] = self._quaternion_to_rotation_matrix(orientation)
            
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
        
        return self.move_to_joint_positions(joint_angles, velocity_scale, acceleration_scale, blocking)
    
    def move_linear(self, target_position: np.ndarray, 
                   target_orientation: np.ndarray,
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   blocking: bool = True) -> bool:
        """直线运动到目标位姿"""
        if not self.is_ready():
            return False
        
        current_position, current_orientation = self.get_end_effector_pose()
        
        distance = np.linalg.norm(target_position - current_position)
        num_points = max(20, int(distance / 0.002))  # 每2mm一个点，更精细
        
        for i in range(num_points + 1):
            t = i / num_points
            
            interp_position = current_position + t * (target_position - current_position)
            interp_orientation = self._slerp_quaternion(current_orientation, target_orientation, t)
            
            if not self.move_to_pose(interp_position, interp_orientation, 
                                   velocity_scale=velocity, blocking=True):
                return False
        
        return True
    
    # ===================== 力控制功能 =====================
    
    def set_joint_torques(self, torques: np.ndarray) -> bool:
        """设置关节力矩"""
        if not self._validate_joint_torques(torques):
            return False
        
        self._joint_torques = torques.copy()
        print(f"Franka设置关节力矩: {torques}")
        return True
    
    def set_cartesian_force(self, force: np.ndarray, torque: np.ndarray) -> bool:
        """设置笛卡尔空间力/力矩"""
        if len(force) != 3 or len(torque) != 3:
            return False
        
        print(f"Franka设置笛卡尔力: {force}, 力矩: {torque}")
        return True
    
    def enable_impedance_control(self, stiffness: np.ndarray, damping: np.ndarray) -> bool:
        """
        启用阻抗控制
        
        Args:
            stiffness: 刚度参数 (6x1: [kx, ky, kz, krx, kry, krz])
            damping: 阻尼参数 (6x1)
            
        Returns:
            是否成功启用
        """
        if len(stiffness) != 6 or len(damping) != 6:
            return False
        
        self._impedance_control = True
        print(f"Franka启用阻抗控制 - 刚度: {stiffness}, 阻尼: {damping}")
        return True
    
    def disable_impedance_control(self) -> bool:
        """禁用阻抗控制"""
        self._impedance_control = False
        print("Franka禁用阻抗控制")
        return True
    
    # ===================== 安全功能 =====================
    
    def enable_collision_detection(self) -> bool:
        """启用碰撞检测"""
        self._collision_detection = True
        print("Franka启用碰撞检测")
        return True
    
    def disable_collision_detection(self) -> bool:
        """禁用碰撞检测"""
        self._collision_detection = False
        print("Franka禁用碰撞检测")
        return True
    
    def set_collision_thresholds(self, joint_torque_thresholds: np.ndarray,
                               cartesian_force_thresholds: np.ndarray) -> bool:
        """
        设置碰撞检测阈值
        
        Args:
            joint_torque_thresholds: 关节力矩阈值
            cartesian_force_thresholds: 笛卡尔力阈值
            
        Returns:
            设置是否成功
        """
        if len(joint_torque_thresholds) != 7 or len(cartesian_force_thresholds) != 6:
            return False
        
        print(f"Franka设置碰撞阈值 - 关节力矩: {joint_torque_thresholds}, 笛卡尔力: {cartesian_force_thresholds}")
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
        
        if hasattr(self._ik_solver, 'use_pinocchio') and self._ik_solver.use_pinocchio:
            return self._ik_solver.ik_pinocchio_method(position, orientation, current_joint_angles)
        else:
            return self._ik_solver.ik_optimization_method(position, orientation, current_joint_angles)
    
    # ===================== 末端执行器控制 =====================
    
    def get_end_effector_type(self) -> EndEffectorType:
        """获取末端执行器类型"""
        return EndEffectorType.GRIPPER
    
    def open_gripper(self, width: float = 0.08) -> bool:
        """
        打开夹爪
        
        Args:
            width: 夹爪宽度 (米)
            
        Returns:
            执行是否成功
        """
        if width < 0 or width > 0.08:
            return False
        
        self._gripper_width = width
        self._gripper_grasping = False
        print(f"Franka夹爪打开到 {width*1000:.1f}mm")
        return True
    
    def close_gripper(self, force: float = 20.0) -> bool:
        """
        关闭夹爪
        
        Args:
            force: 夹持力 (牛顿)
            
        Returns:
            执行是否成功
        """
        if force < 0 or force > 70.0:  # Franka夹爪最大力70N
            return False
        
        self._gripper_width = 0.0
        self._gripper_grasping = True
        print(f"Franka夹爪以 {force:.1f}N 力度关闭")
        return True
    
    def get_gripper_state(self) -> Dict:
        """获取夹爪状态"""
        return {
            "type": "parallel_gripper",
            "available": True,
            "width": self._gripper_width,
            "max_width": 0.08,
            "force": 20.0 if self._gripper_grasping else 0.0,
            "max_force": 70.0,
            "is_grasping": self._gripper_grasping,
            "temperature": 25.0  # 模拟温度
        }
    
    def grasp(self, width: float = 0.05, force: float = 20.0, epsilon_inner: float = 0.01, 
             epsilon_outer: float = 0.01) -> bool:
        """
        执行抓取动作
        
        Args:
            width: 目标宽度
            force: 夹持力
            epsilon_inner: 内部epsilon
            epsilon_outer: 外部epsilon
            
        Returns:
            抓取是否成功
        """
        print(f"Franka执行抓取 - 宽度: {width}m, 力: {force}N")
        # 简化的抓取逻辑
        self._gripper_width = width
        self._gripper_grasping = True
        return True
    
    # ===================== 状态和诊断 =====================
    
    def get_robot_state_detailed(self) -> Dict:
        """获取详细的机器人状态"""
        base_state = self.get_status_report()
        
        franka_specific = {
            "collision_detection_enabled": self._collision_detection,
            "impedance_control_enabled": self._impedance_control,
            "gripper_state": self.get_gripper_state(),
            "joint_torques": self._joint_torques.tolist(),
            "torque_limits": self.torque_limits.tolist()
        }
        
        base_state.update(franka_specific)
        return base_state
    
    # ===================== 内部工具方法 =====================
    
    def _validate_joint_torques(self, torques: np.ndarray) -> bool:
        """验证关节力矩"""
        if len(torques) != self.dof:
            return False
        
        limits = self.torque_limits
        return np.all(np.abs(torques) <= limits)
    
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
        
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2


class FrankaHardware(FrankaSimRobot):
    """
    Franka Emika真实硬件接口
    """
    
    def __init__(self, robot_id: str = "franka_hardware",
                 franka_ip: str = "172.16.0.2"):
        """
        初始化Franka硬件接口
        
        Args:
            robot_id: 机械臂标识符
            franka_ip: Franka控制器IP地址
        """
        super().__init__(robot_id)
        self.franka_ip = franka_ip
        self._robot = None
        self._hardware_connected = False
    
    def connect(self) -> bool:
        """连接Franka硬件"""
        try:
            # 这里应该使用libfranka或franka_ros连接
            # import franka
            # self._robot = franka.Robot(self.franka_ip)
            
            print(f"正在连接Franka硬件 ({self.franka_ip})...")
            
            # 模拟连接
            self._hardware_connected = True
            self._set_state(self.state.__class__.IDLE)
            
            print("Franka硬件连接成功")
            return True
            
        except Exception as e:
            self._set_state(self.state.__class__.ERROR, f"Franka硬件连接失败: {str(e)}")
            return False
    
    def disconnect(self) -> bool:
        """断开Franka硬件连接"""
        try:
            self._robot = None
            self._hardware_connected = False
            self._set_state(self.state.__class__.DISABLED)
            print("Franka硬件已断开连接")
            return True
        except Exception as e:
            return False
    
    def is_ready(self) -> bool:
        """检查Franka硬件是否就绪"""
        return self._hardware_connected and super().is_ready() 