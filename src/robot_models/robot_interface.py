"""
机械臂抽象接口
定义标准的机械臂操作接口，支持不同品牌的机械臂
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Tuple, Union
import numpy as np
from enum import Enum
import os


class RobotType(Enum):
    """机械臂类型枚举"""
    UNITREE_D1 = "unitree_d1"
    UR3E = "ur3e"
    UR5 = "ur5"
    UR10 = "ur10"
    FRANKA_PANDA = "franka_panda"
    KUKA_IIWA = "kuka_iiwa"
    ABB_IRB = "abb_irb"
    CUSTOM = "custom"


class RobotState(Enum):
    """机械臂状态枚举"""
    IDLE = "idle"
    MOVING = "moving"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"
    DISABLED = "disabled"


class EndEffectorType(Enum):
    """末端执行器类型"""
    NONE = "none"
    GRIPPER = "gripper"
    VACUUM = "vacuum"
    WELDING_TORCH = "welding_torch"
    CUSTOM = "custom"


class RobotInterface(ABC):
    """
    机械臂抽象接口基类
    所有机械臂实现都应继承此接口
    """
    
    def __init__(self, robot_type: RobotType, robot_id: str = "robot_0"):
        """
        初始化机械臂接口
        
        Args:
            robot_type: 机械臂类型
            robot_id: 机械臂唯一标识符
        """
        self.robot_type = robot_type
        self.robot_id = robot_id
        self._state = RobotState.IDLE
        self._error_message = ""
        
    # ===================== 基本属性 =====================
    
    @property
    @abstractmethod
    def name(self) -> str:
        """机械臂名称"""
        pass
    
    @property
    @abstractmethod
    def dof(self) -> int:
        """自由度数量"""
        pass
    
    @property
    @abstractmethod
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        pass
    
    @property
    @abstractmethod
    def joint_limits(self) -> np.ndarray:
        """关节限位，形状为(n_joints, 2)，每行包含[min, max]"""
        pass
    
    @property
    @abstractmethod
    def velocity_limits(self) -> np.ndarray:
        """关节速度限位"""
        pass
    
    @property
    @abstractmethod
    def acceleration_limits(self) -> np.ndarray:
        """关节加速度限位"""
        pass
    
    @property
    def state(self) -> RobotState:
        """当前状态"""
        return self._state
    
    @property
    def error_message(self) -> str:
        """错误信息"""
        return self._error_message
    
    # ===================== DH参数和运动学 =====================
    
    @abstractmethod
    def get_dh_params(self) -> np.ndarray:
        """
        获取DH参数
        
        Returns:
            DH参数矩阵，形状为(n_joints, 4)
            每行包含[a, alpha, d, theta]
        """
        pass
    
    @abstractmethod
    def get_base_transform(self) -> np.ndarray:
        """
        获取基座变换矩阵
        
        Returns:
            4x4变换矩阵
        """
        pass
    
    @abstractmethod
    def get_end_effector_transform(self) -> np.ndarray:
        """
        获取末端执行器相对于最后一个关节的变换
        
        Returns:
            4x4变换矩阵
        """
        pass
    
    # ===================== 状态获取 =====================
    
    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        """
        获取当前关节位置
        
        Returns:
            关节位置数组
        """
        pass
    
    @abstractmethod
    def get_joint_velocities(self) -> np.ndarray:
        """
        获取当前关节速度
        
        Returns:
            关节速度数组
        """
        pass
    
    @abstractmethod
    def get_joint_torques(self) -> np.ndarray:
        """
        获取当前关节力矩
        
        Returns:
            关节力矩数组
        """
        pass
    
    @abstractmethod
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取末端执行器位姿
        
        Returns:
            position: 位置向量(3,)
            orientation: 四元数(4,) [w, x, y, z]
        """
        pass
    
    # ===================== 运动控制 =====================
    
    @abstractmethod
    def move_to_joint_positions(self, positions: np.ndarray, 
                               velocity_scale: float = 1.0,
                               acceleration_scale: float = 1.0,
                               blocking: bool = True) -> bool:
        """
        移动到指定关节位置
        
        Args:
            positions: 目标关节位置
            velocity_scale: 速度缩放因子
            acceleration_scale: 加速度缩放因子
            blocking: 是否阻塞等待完成
            
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def move_to_pose(self, position: np.ndarray, orientation: np.ndarray,
                    velocity_scale: float = 1.0,
                    acceleration_scale: float = 1.0,
                    blocking: bool = True) -> bool:
        """
        移动到指定末端位姿
        
        Args:
            position: 目标位置
            orientation: 目标姿态（四元数）
            velocity_scale: 速度缩放因子
            acceleration_scale: 加速度缩放因子
            blocking: 是否阻塞等待完成
            
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def move_linear(self, target_position: np.ndarray, 
                   target_orientation: np.ndarray,
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   blocking: bool = True) -> bool:
        """
        直线运动到目标位姿
        
        Args:
            target_position: 目标位置
            target_orientation: 目标姿态（四元数）
            velocity: 运动速度 (m/s)
            acceleration: 运动加速度 (m/s²)
            blocking: 是否阻塞等待完成
            
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def stop_motion(self, emergency: bool = False) -> bool:
        """
        停止运动
        
        Args:
            emergency: 是否紧急停止
            
        Returns:
            执行是否成功
        """
        pass
    
    # ===================== 速度控制 =====================
    
    def set_joint_velocities(self, velocities: np.ndarray) -> bool:
        """
        设置关节速度（实时控制）
        
        Args:
            velocities: 关节速度数组
            
        Returns:
            执行是否成功
        """
        # 默认实现，子类可以重写
        return False
    
    def set_cartesian_velocity(self, linear_velocity: np.ndarray, 
                              angular_velocity: np.ndarray) -> bool:
        """
        设置笛卡尔空间速度
        
        Args:
            linear_velocity: 线速度向量
            angular_velocity: 角速度向量
            
        Returns:
            执行是否成功
        """
        # 默认实现，子类可以重写
        return False
    
    # ===================== 力控制 =====================
    
    def set_joint_torques(self, torques: np.ndarray) -> bool:
        """
        设置关节力矩
        
        Args:
            torques: 关节力矩数组
            
        Returns:
            执行是否成功
        """
        # 默认实现，子类可以重写
        return False
    
    def set_cartesian_force(self, force: np.ndarray, torque: np.ndarray) -> bool:
        """
        设置笛卡尔空间力/力矩
        
        Args:
            force: 力向量
            torque: 力矩向量
            
        Returns:
            执行是否成功
        """
        # 默认实现，子类可以重写
        return False
    
    # ===================== 配置和校准 =====================
    
    @abstractmethod
    def home(self) -> bool:
        """
        回零位
        
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def calibrate(self) -> bool:
        """
        校准机械臂
        
        Returns:
            校准是否成功
        """
        pass
    
    # ===================== 安全和状态管理 =====================
    
    @abstractmethod
    def enable(self) -> bool:
        """
        使能机械臂
        
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def disable(self) -> bool:
        """
        失能机械臂
        
        Returns:
            执行是否成功
        """
        pass
    
    @abstractmethod
    def is_ready(self) -> bool:
        """
        检查机械臂是否就绪
        
        Returns:
            是否就绪
        """
        pass
    
    @abstractmethod
    def is_moving(self) -> bool:
        """
        检查机械臂是否在运动
        
        Returns:
            是否在运动
        """
        pass
    
    def get_state(self) -> RobotState:
        """
        获取机械臂当前状态
        
        Returns:
            机械臂状态
        """
        return self._state
    
    def check_joint_limits(self, positions: np.ndarray) -> bool:
        """
        检查关节位置是否在限位内
        
        Args:
            positions: 关节位置
            
        Returns:
            是否在限位内
        """
        limits = self.joint_limits
        return np.all(positions >= limits[:, 0]) and np.all(positions <= limits[:, 1])
    
    def check_velocity_limits(self, velocities: np.ndarray) -> bool:
        """
        检查关节速度是否在限位内
        
        Args:
            velocities: 关节速度
            
        Returns:
            是否在限位内
        """
        limits = self.velocity_limits
        return np.all(np.abs(velocities) <= limits)
    
    # ===================== 末端执行器控制 =====================
    
    def get_end_effector_type(self) -> EndEffectorType:
        """获取末端执行器类型"""
        return EndEffectorType.NONE
    
    def open_gripper(self, width: float = 1.0) -> bool:
        """
        打开夹爪
        
        Args:
            width: 打开宽度（归一化，0-1）
            
        Returns:
            执行是否成功
        """
        return False
    
    def close_gripper(self, force: float = 1.0) -> bool:
        """
        关闭夹爪
        
        Args:
            force: 夹持力（归一化，0-1）
            
        Returns:
            执行是否成功
        """
        return False
    
    def get_gripper_state(self) -> Dict:
        """
        获取夹爪状态
        
        Returns:
            夹爪状态字典
        """
        return {"type": "none", "available": False}
    
    # ===================== 工具和坐标系 =====================
    
    def set_tool_transform(self, transform: np.ndarray) -> bool:
        """
        设置工具坐标系变换
        
        Args:
            transform: 4x4变换矩阵
            
        Returns:
            设置是否成功
        """
        return False
    
    def set_base_transform(self, transform: np.ndarray) -> bool:
        """
        设置基座坐标系变换
        
        Args:
            transform: 4x4变换矩阵
            
        Returns:
            设置是否成功
        """
        return False
    
    # ===================== 高级功能 =====================
    
    def get_jacobian(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """
        获取雅可比矩阵
        
        Args:
            joint_positions: 关节位置，None表示当前位置
            
        Returns:
            雅可比矩阵
        """
        # 默认实现，子类可以重写提供更高效的实现
        return np.zeros((6, self.dof))
    
    def solve_ik(self, target_pose: np.ndarray, current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        求解逆运动学
        
        Args:
            target_pose: 4x4目标位姿矩阵
            current_joint_angles: 当前关节角度（弧度），可以为None
            
        Returns:
            joint_angles: 关节角度
            success: 是否成功
        """
        # 默认实现，子类可以重写提供更高效的实现
        if current_joint_angles is None:
            current_joint_angles = np.zeros(self.dof)
        return current_joint_angles, False
    
    # ===================== 信息和调试 =====================
    
    def get_robot_info(self) -> Dict:
        """
        获取机械臂信息
        
        Returns:
            机械臂信息字典
        """
        return {
            "name": self.name,
            "type": self.robot_type.value,
            "id": self.robot_id,
            "dof": self.dof,
            "joint_names": self.joint_names,
            "state": self.state.value,
            "error_message": self.error_message
        }
    
    def get_status_report(self) -> Dict:
        """
        获取详细状态报告
        
        Returns:
            状态报告字典
        """
        try:
            joint_pos = self.get_joint_positions()
            joint_vel = self.get_joint_velocities()
            ee_pos, ee_orient = self.get_end_effector_pose()
            
            return {
                "timestamp": np.datetime64('now').astype(str),
                "robot_info": self.get_robot_info(),
                "joint_positions": joint_pos.tolist(),
                "joint_velocities": joint_vel.tolist(),
                "end_effector_position": ee_pos.tolist(),
                "end_effector_orientation": ee_orient.tolist(),
                "is_ready": self.is_ready(),
                "is_moving": self.is_moving(),
                "gripper_state": self.get_gripper_state()
            }
        except Exception as e:
            return {
                "timestamp": np.datetime64('now').astype(str),
                "error": f"Failed to get status: {str(e)}"
            }
    
    # ===================== 内部工具方法 =====================
    
    def _set_state(self, state: RobotState, error_message: str = ""):
        """设置机械臂状态"""
        self._state = state
        self._error_message = error_message
    
    def _validate_joint_positions(self, positions: np.ndarray) -> bool:
        """验证关节位置"""
        if len(positions) != self.dof:
            self._set_state(RobotState.ERROR, f"关节位置数量不匹配: 期望{self.dof}，实际{len(positions)}")
            return False
        
        # 使用numpy数组进行限位检查（避免tensor操作问题）
        limits_np = self.joint_limits
        if not (np.all(positions >= limits_np[:, 0]) and np.all(positions <= limits_np[:, 1])):
            print(f"关节位置超出限位:")
            print(f"  位置: {positions}")
            print(f"  下限: {limits_np[:, 0]}")
            print(f"  上限: {limits_np[:, 1]}")
            # 不设置为错误状态，只是警告
            return True  # 允许继续，但给出警告
        
        return True
    
    def _validate_joint_velocities(self, velocities: np.ndarray) -> bool:
        """验证关节速度"""
        if len(velocities) != self.dof:
            self._set_state(RobotState.ERROR, f"关节速度数量不匹配: 期望{self.dof}，实际{len(velocities)}")
            return False
        
        # 检查是否包含NaN值
        if np.any(np.isnan(velocities)):
            print(f"⚠️ 检测到NaN速度值: {velocities}")
            self._set_state(RobotState.ERROR, "关节速度包含NaN值")
            return False
        
        # 检查是否包含无穷大值
        if np.any(np.isinf(velocities)):
            print(f"⚠️ 检测到无穷大速度值: {velocities}")
            self._set_state(RobotState.ERROR, "关节速度包含无穷大值")
            return False
        
        if not self.check_velocity_limits(velocities):
            print(f"⚠️ 关节速度超出限位:")
            print(f"  速度: {velocities}")
            print(f"  限制: {self.velocity_limits}")
            self._set_state(RobotState.ERROR, "关节速度超出限位")
            return False
        
        return True


class SimulatedRobotInterface(RobotInterface):
    """
    基于Isaac Gym的机械臂仿真接口
    使用URDF文件创建仿真环境，支持单环境高性能仿真
    """
    
    def __init__(self, robot_type: RobotType, urdf_path: str, 
                 robot_id: str = "isaac_robot", headless: bool = False,
                 sim_params: Optional[Dict] = None, device: str = "cuda:0"):
        """
        初始化Isaac Gym仿真环境
        
        Args:
            robot_type: 机械臂类型
            urdf_path: URDF文件路径
            robot_id: 机械臂唯一标识符
            headless: 是否无界面模式
            sim_params: 仿真参数字典
            device: 计算设备 ("cuda:0" 或 "cpu")
        """
        super().__init__(robot_type, robot_id)
        
        try:
            from isaacgym import gymapi, gymutil, gymtorch
            import torch
        except ImportError:
            raise ImportError("Isaac Gym未安装，请按照NVIDIA Isaac Gym文档安装")
        
        self.gym = gymapi.acquire_gym()
        self.gymtorch = gymtorch
        self.torch = torch
        self.urdf_path = urdf_path
        self.headless = headless
        self.device = device
        
        # 解析设备信息
        if device.startswith('cuda'):
            self.sim_device_id = int(device.split(':')[1]) if ':' in device else 0
            self.sim_device_type = 'cuda'
        else:
            self.sim_device_id = 0
            self.sim_device_type = 'cpu'
            
        # 仿真参数
        self.sim_params = self._parse_sim_params(sim_params)
        self.sim_dt = self.sim_params.dt
            
        # Isaac Gym对象
        self.sim = None
        self.env = None
        self.robot_handle = None
        self.robot_asset = None
        self.viewer = None
        
        # 状态tensor
        self.dof_state_tensor = None
        self.rigid_body_state_tensor = None
        self.jacobian_tensor = None
        self.mass_matrix_tensor = None
        
        # 机械臂属性
        self._enabled = False
        
        # 初始化仿真环境
        self._setup_sim()
        self._load_robot_asset()
        self._create_env()
        self._setup_viewer()
        self._prepare_sim_tensors()
        
        print(f"✅ Isaac Gym仿真环境初始化完成")
        print(f"   机械臂: {self.name}")
        print(f"   自由度: {self.dof}")
        print(f"   设备: {self.device}")
        
    def _parse_sim_params(self, custom_params: Optional[Dict] = None):
        """解析仿真参数"""
        from isaacgym import gymapi
        
        sim_params = gymapi.SimParams()
        
        # 通用参数
        sim_params.dt = 1.0 / 60.0  # 60 FPS
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
        sim_params.num_client_threads = 0
        
        # PhysX参数
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.rest_offset = 0.0
        sim_params.physx.contact_offset = 0.001
        sim_params.physx.friction_offset_threshold = 0.001
        sim_params.physx.friction_correlation_distance = 0.0005
        sim_params.physx.num_threads = 1
        sim_params.physx.use_gpu = True
        sim_params.physx.num_subscenes = 0
        
        # GPU管线 - 启用以获得更好性能，但需要使用tensor API
        sim_params.use_gpu_pipeline = (self.sim_device_type == 'cuda')
        
        # 注意：当use_gpu_pipeline=True时，必须使用tensor API：
        # - 使用 gym.set_dof_position_target_tensor() 而非 gym.set_actor_dof_position_targets()
        # - 使用 gym.set_dof_velocity_target_tensor() 而非 gym.set_actor_dof_velocity_targets()
        
        # 应用自定义参数
        if custom_params:
            for key, value in custom_params.items():
                if hasattr(sim_params, key):
                    setattr(sim_params, key, value)
                    
        return sim_params
    
    def _setup_sim(self):
        """创建仿真环境"""
        from isaacgym import gymapi
        
        # 设置图形设备
        graphics_device_id = self.sim_device_id if not self.headless else -1
        
        self.sim = self.gym.create_sim(
            self.sim_device_id, graphics_device_id, 
            gymapi.SIM_PHYSX, self.sim_params
        )
        
        if self.sim is None:
            raise RuntimeError("创建Isaac Gym仿真环境失败")
            
        # 创建地面
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        plane_params.static_friction = 1.0
        plane_params.dynamic_friction = 1.0
        plane_params.restitution = 0.0
        self.gym.add_ground(self.sim, plane_params)
        
    def _load_robot_asset(self):
        """加载机械臂资源"""
        from isaacgym import gymapi
        
        # 解析URDF路径
        asset_root = os.path.dirname(self.urdf_path)
        asset_file = os.path.basename(self.urdf_path)
        
        # 资源加载选项
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.flip_visual_attachments = True
        asset_options.collapse_fixed_joints = False
        asset_options.disable_gravity = False
        asset_options.thickness = 0.001
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        asset_options.armature = 0.01
        asset_options.linear_damping = 0.01
        asset_options.angular_damping = 0.01
        
        # 加载资源
        self.robot_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        
        if self.robot_asset is None:
            raise RuntimeError(f"加载URDF文件失败: {self.urdf_path}")
        
        # 获取机械臂信息
        self._dof_count = self.gym.get_asset_dof_count(self.robot_asset)
        self._body_count = self.gym.get_asset_rigid_body_count(self.robot_asset)
        
        # 获取DOF属性和限位
        dof_props = self.gym.get_asset_dof_properties(self.robot_asset)
        
        # 先创建numpy数组，然后转换为tensor
        joint_limits_np = np.zeros((self._dof_count, 2), dtype=np.float32)
        velocity_limits_np = np.zeros(self._dof_count, dtype=np.float32)
        torque_limits_np = np.zeros(self._dof_count, dtype=np.float32)
        
        for i in range(self._dof_count):
            joint_limits_np[i, 0] = float(dof_props['lower'][i])
            joint_limits_np[i, 1] = float(dof_props['upper'][i])
            velocity_limits_np[i] = float(dof_props['velocity'][i])
            torque_limits_np[i] = float(dof_props['effort'][i])
        
        # 对于UNITREE_D1，设置合理的速度限制
        if self.robot_type == RobotType.UNITREE_D1:
            # 检查是否有无效的速度限制值
            for i in range(self._dof_count):
                if velocity_limits_np[i] <= 0 or np.isinf(velocity_limits_np[i]) or np.isnan(velocity_limits_np[i]):
                    velocity_limits_np[i] = 2.0  # 设置为合理的默认值 2.0 rad/s
            print(f"为D1机械臂设置速度限制: {velocity_limits_np}")
        
        # 转换为tensor
        self._joint_limits = self.torch.from_numpy(joint_limits_np).to(self.device)
        self._velocity_limits = self.torch.from_numpy(velocity_limits_np).to(self.device)
        self._torque_limits = self.torch.from_numpy(torque_limits_np).to(self.device)
        
        # 获取关节名称
        self._joint_names = self.gym.get_asset_dof_names(self.robot_asset)
        self._body_names = self.gym.get_asset_rigid_body_names(self.robot_asset)
        
        print(f"✅ 加载资源: {asset_file}")
        print(f"   关节数: {self._dof_count}")
        print(f"   刚体数: {self._body_count}")
        
    def _create_env(self):
        """创建单个环境"""
        from isaacgym import gymapi
        
        # 创建环境
        env_lower = gymapi.Vec3(-2.0, -2.0, 0.0)
        env_upper = gymapi.Vec3(2.0, 2.0, 2.0)
        
        self.env = self.gym.create_env(self.sim, env_lower, env_upper, 1)
        
        # 创建机械臂actor
        start_pose = gymapi.Transform()
        start_pose.p = gymapi.Vec3(0.0, 0.0, 0.0)
        start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        
        self.robot_handle = self.gym.create_actor(
            self.env, self.robot_asset, start_pose, 
            f"robot_{self.robot_id}", 0, 0
        )
        
        # 设置DOF属性
        dof_props = self.gym.get_actor_dof_properties(self.env, self.robot_handle)
        self.gym.set_actor_dof_properties(self.env, self.robot_handle, dof_props)
        
        # 设置刚体属性
        body_props = self.gym.get_actor_rigid_body_properties(self.env, self.robot_handle)
        self.gym.set_actor_rigid_body_properties(
            self.env, self.robot_handle, body_props, recomputeInertia=True
        )
    
    def _prepare_sim_tensors(self):
        """准备仿真tensor（优化版，防止nan值）"""
        # 准备仿真
        self.gym.prepare_sim(self.sim)
        
        # *** 关键修复1: 设置安全的初始关节状态 ***
        print("设置初始关节状态...")
        self._set_safe_initial_joint_positions()
        
        # *** 关键修复2: 运行稳定化步骤 ***
        print("运行稳定化步骤...")
        for i in range(30):  # 运行30步让物理引擎稳定
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)
        
        # 刷新tensor（必须在acquire之前）
        self._refresh_sim_tensors()
        
        # 获取状态tensor
        dof_state_tensor = self.gym.acquire_dof_state_tensor(self.sim)
        rigid_body_state_tensor = self.gym.acquire_rigid_body_state_tensor(self.sim)
        net_contact_forces_tensor = self.gym.acquire_net_contact_force_tensor(self.sim)
        
        # 包装tensor
        self.dof_state = self.gymtorch.wrap_tensor(dof_state_tensor)
        self.rigid_body_states = self.gymtorch.wrap_tensor(rigid_body_state_tensor)
        self.contact_forces = self.gymtorch.wrap_tensor(net_contact_forces_tensor)
        
        # 获取雅可比和质量矩阵tensor
        try:
            jacobian_tensor = self.gym.acquire_jacobian_tensor(self.sim, f"robot_{self.robot_id}")
            mass_matrix_tensor = self.gym.acquire_mass_matrix_tensor(self.sim, f"robot_{self.robot_id}")
            
            self.jacobian = self.gymtorch.wrap_tensor(jacobian_tensor)
            self.mass_matrix = self.gymtorch.wrap_tensor(mass_matrix_tensor)
        except:
            # 如果获取失败，创建默认tensor
            self.jacobian = self.torch.zeros((6, self._dof_count), device=self.device)
            self.mass_matrix = self.torch.zeros((self._dof_count, self._dof_count), device=self.device)
        
        # 分离位置和速度
        # DOF状态tensor格式：[pos1, vel1, pos2, vel2, ...] 交替排列
        self.dof_pos = self.dof_state.view(-1, 2)[:, 0]  # 位置
        self.dof_vel = self.dof_state.view(-1, 2)[:, 1]  # 速度
        
        # 重塑刚体状态 (每个刚体13个状态：pos(3) + quat(4) + vel(3) + ang_vel(3))
        self.rigid_body_pos = self.rigid_body_states.view(-1, 13)[:self._body_count, 0:3]
        self.rigid_body_rot = self.rigid_body_states.view(-1, 13)[:self._body_count, 3:7]
        self.rigid_body_vel = self.rigid_body_states.view(-1, 13)[:self._body_count, 7:10]
        self.rigid_body_ang_vel = self.rigid_body_states.view(-1, 13)[:self._body_count, 10:13]
        
        # *** 关键修复3: 验证状态并报告 ***
        current_pos = self.dof_pos.cpu().numpy()
        if self.torch.isnan(self.dof_pos).any():
            print("❌ 警告: 关节位置仍包含NaN值!")
            nan_indices = self.torch.where(self.torch.isnan(self.dof_pos))[0]
            print(f"   NaN位置索引: {nan_indices.cpu().numpy()}")
        else:
            print("✅ 关节位置数值正常")
        
        print(f"✅ 仿真tensor准备完成")
        print(f"   DOF状态: {self.dof_state.shape}")
        print(f"   刚体状态: {self.rigid_body_states.shape}")
        print(f"   当前关节位置: {current_pos}")
        
    def _refresh_sim_tensors(self):
        """刷新仿真tensor"""
        self.gym.refresh_dof_state_tensor(self.sim)
        self.gym.refresh_rigid_body_state_tensor(self.sim)
        self.gym.refresh_net_contact_force_tensor(self.sim)
        
        try:
            self.gym.refresh_jacobian_tensors(self.sim)
            self.gym.refresh_mass_matrix_tensors(self.sim)
        except:
            pass
    
    def _set_safe_initial_joint_positions(self):
        """设置安全的初始关节位置，防止nan值（特别针对D1机械臂优化）"""
        import numpy as np
        
        # 创建安全的初始位置
        initial_positions = np.zeros(self._dof_count, dtype=np.float32)
        
        # 获取关节限位
        limits_array = self._joint_limits.cpu().numpy()
        
        for i in range(self._dof_count):
            lower_limit = limits_array[i, 0]
            upper_limit = limits_array[i, 1]
            
            if np.isfinite(lower_limit) and np.isfinite(upper_limit):
                # 特殊处理夹爪关节（针对D1机械臂）
                if i >= 6:  # 夹爪关节 (关节7和8)
                    # 夹爪设置为安全的非边界位置
                    if lower_limit >= 0:  # 关节7: [0, 0.03]
                        initial_positions[i] = 0.01  # 稍微张开，避开边界
                    elif upper_limit <= 0:  # 关节8: [-0.03, 0]
                        initial_positions[i] = -0.01  # 稍微张开，避开边界  
                    else:
                        initial_positions[i] = (lower_limit + upper_limit) / 2
                else:
                    # 主要关节设置为零位（如果零位在限位内）
                    if lower_limit <= 0.0 <= upper_limit:
                        initial_positions[i] = 0.0
                    else:
                        # 如果零位不在限位内，设置为中间位置
                        initial_positions[i] = (lower_limit + upper_limit) / 2
            else:
                initial_positions[i] = 0.0
            
            # 再次检查并约束到限位内
            initial_positions[i] = np.clip(initial_positions[i], lower_limit, upper_limit)
        
        print(f"初始关节位置: {initial_positions}")
        
        # 使用tensor API设置初始位置（避免GPU流水线错误）
        targets_tensor = self.torch.tensor(initial_positions, dtype=self.torch.float32, device=self.device)
        self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
    
    def _clamp_to_joint_limits(self, positions: np.ndarray) -> np.ndarray:
        """将关节位置限制到安全范围内"""
        import numpy as np
        
        if len(positions) != self._dof_count:
            print(f"❌ 位置数组长度({len(positions)}) 与自由度({self._dof_count})不匹配")
            return positions
        
        # 获取关节限位
        limits_array = self._joint_limits.cpu().numpy()
        clamped_positions = positions.copy()
        
        for i in range(len(positions)):
            lower_limit = limits_array[i, 0]
            upper_limit = limits_array[i, 1]
            original_pos = positions[i]
            
            # 限制到安全范围
            clamped_pos = np.clip(positions[i], lower_limit, upper_limit)
            clamped_positions[i] = clamped_pos
            
            # 报告显著的修正
            if abs(original_pos - clamped_pos) > 1e-4:
                print(f"   关节{i+1}: {original_pos:.4f} → {clamped_pos:.4f} "
                      f"(限位: [{lower_limit:.3f}, {upper_limit:.3f}])")
        
        return clamped_positions
    
    def _setup_viewer(self):
        """设置可视化窗口"""
        if not self.headless:
            from isaacgym import gymapi
            
            self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
            if self.viewer is None:
                print("警告: 无法创建可视化窗口")
                return
                
            # 订阅键盘事件
            self.gym.subscribe_viewer_keyboard_event(self.viewer, gymapi.KEY_ESCAPE, "QUIT")
            self.gym.subscribe_viewer_keyboard_event(self.viewer, gymapi.KEY_R, "RESET")
            
            # 设置相机位置
            if self.sim_params.up_axis == gymapi.UP_AXIS_Z:
                cam_pos = gymapi.Vec3(2.0, 2.0, 1.5)
                cam_target = gymapi.Vec3(0.0, 0.0, 0.5)
            else:
                cam_pos = gymapi.Vec3(2.0, 1.5, 2.0)
                cam_target = gymapi.Vec3(0.0, 0.0, 1.0)
                
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)
    
    def _update_state(self):
        """更新机械臂状态"""
        if not self.sim:
            return
        
        # 刷新tensor数据
        self._refresh_sim_tensors()
    
    @property
    def dof(self) -> int:
        """自由度数量"""
        return self._dof_count
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        return self._joint_names
    
    @property
    def joint_limits(self) -> np.ndarray:
        """关节限位"""
        return self._joint_limits.cpu().numpy()
    
    @property
    def velocity_limits(self) -> np.ndarray:
        """关节速度限位"""
        return self._velocity_limits.cpu().numpy()
    
    @property
    def acceleration_limits(self) -> np.ndarray:
        """关节加速度限位"""
        # Isaac Gym中没有直接的加速度限位，返回默认值
        return np.ones(self.dof) * 10.0
    
    @property
    def name(self) -> str:
        """机械臂名称"""
        return f"Isaac Gym {self.robot_type.value}"
    
    def get_dh_params(self) -> np.ndarray:
        """
        获取DH参数 - Isaac Gym中从URDF推导
        注意：这是一个简化实现，实际应该从URDF解析
        """
        # 返回默认DH参数矩阵，实际应该从URDF文件解析
        return np.zeros((self.dof, 4))
    
    def get_base_transform(self) -> np.ndarray:
        """获取基座变换矩阵"""
        return np.eye(4)
    
    def get_end_effector_transform(self) -> np.ndarray:
        """获取末端执行器相对于最后一个关节的变换"""
        return np.eye(4)
    
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取末端执行器位姿
        """
        try:
            self._update_state()
            
            # 假设最后一个刚体是末端执行器
            ee_pos = self.rigid_body_pos[-1].cpu().numpy()
            ee_quat = self.rigid_body_rot[-1].cpu().numpy()
            
            return ee_pos, ee_quat
            
        except Exception as e:
            print(f"获取末端执行器位姿失败: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])
    
    def move_to_pose(self, position: np.ndarray, orientation: np.ndarray,
                    velocity_scale: float = 1.0,
                    acceleration_scale: float = 1.0,
                    blocking: bool = True) -> bool:
        """
        移动到指定末端位姿 - 需要逆运动学求解
        """
        # 简化实现：使用逆运动学求解关节角度
        # 构造4x4位姿矩阵
        target_pose = np.eye(4)
        target_pose[:3, 3] = position
        # 将四元数转换为旋转矩阵
        from scipy.spatial.transform import Rotation as R
        rot = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
        target_pose[:3, :3] = rot.as_matrix()
        
        joint_angles, success = self.solve_ik(target_pose, self.get_joint_positions())
        
        if success:
            return self.move_to_joint_positions(joint_angles, velocity_scale, 
                                              acceleration_scale, blocking)
        else:
            self._set_state(RobotState.ERROR, "逆运动学求解失败")
            return False
    
    def move_linear(self, target_position: np.ndarray, 
                   target_orientation: np.ndarray,
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   blocking: bool = True) -> bool:
        """
        直线运动到目标位姿
        """
        try:
            # 获取当前末端位姿
            current_pos, current_quat = self.get_end_effector_pose()
            
            # 生成线性插值路径
            num_steps = int(np.linalg.norm(target_position - current_pos) / (velocity * self.sim_params.dt))
            num_steps = max(num_steps, 10)  # 至少10步
            
            for i in range(num_steps + 1):
                alpha = i / num_steps
                
                # 位置线性插值
                intermediate_pos = current_pos + alpha * (target_position - current_pos)
                
                # 四元数球面线性插值 (简化为线性插值)
                intermediate_quat = current_quat + alpha * (target_orientation - current_quat)
                intermediate_quat = intermediate_quat / np.linalg.norm(intermediate_quat)
                
                # 移动到中间位姿
                if not self.move_to_pose(intermediate_pos, intermediate_quat, 
                                       velocity_scale=velocity, blocking=True):
                    return False
                
                if not blocking:
                    self.step_simulation(1)
            
            return True
            
        except Exception as e:
            self._set_state(RobotState.ERROR, f"直线运动失败: {str(e)}")
            return False
    
    def solve_ik(self, position: np.ndarray, orientation: np.ndarray,
                initial_guess: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        使用数值方法求解逆运动学
        简化实现，实际项目中建议使用专门的IK求解器
        """
        try:
            from scipy.optimize import minimize
            from isaacgym import gymapi
            
            if initial_guess is None:
                initial_guess = self.get_joint_positions()
            
            def objective(q):
                # 设置关节角度并获取末端位姿（使用tensor API）
                targets_tensor = self.torch.tensor(q.astype(np.float32), dtype=self.torch.float32, device=self.device)
                self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
                self.step_simulation(1)
                
                current_pos, current_quat = self.get_end_effector_pose()
                
                # 位置误差
                pos_error = np.linalg.norm(current_pos - position)
                
                # 姿态误差 (简化为四元数差的模长)
                quat_error = np.linalg.norm(current_quat - orientation)
                
                return pos_error + quat_error
            
            # 关节限位约束
            bounds = [(limits[0], limits[1]) for limits in self.joint_limits]
            
            # 优化求解
            result = minimize(
                objective, initial_guess, 
                method='L-BFGS-B', bounds=bounds,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            success = result.success and result.fun < 1e-3
            return result.x, success
            
        except Exception as e:
            print(f"IK求解失败: {e}")
            return initial_guess if initial_guess is not None else np.zeros(self.dof), False
    
    def get_jacobian(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """
        获取雅可比矩阵
        """
        try:
            self._update_state()
            
            # 如果有内置雅可比矩阵，直接使用
            if hasattr(self, 'jacobian') and self.jacobian.numel() > 0:
                return self.jacobian.cpu().numpy()
            else:
                # 数值计算雅可比矩阵
                return self._compute_jacobian_numerically(joint_positions)
                
        except Exception as e:
            print(f"雅可比矩阵计算失败: {e}")
            return np.zeros((6, self.dof))
    
    def _compute_jacobian_numerically(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """数值计算雅可比矩阵"""
        if joint_positions is None:
            joint_positions = self.get_joint_positions()
        
        jacobian = np.zeros((6, self.dof))
        delta = 1e-6
        
        # 获取当前末端位姿
        current_pos, current_quat = self.get_end_effector_pose()
        
        for i in range(self.dof):
            # 扰动第i个关节
            q_plus = joint_positions.copy()
            q_plus[i] += delta
            
            # 设置关节角度并计算末端位姿
            try:
                targets_tensor = self.torch.tensor(q_plus, dtype=self.torch.float32, device=self.device)
                self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
                self.step_simulation(1)
                
                pos_plus, quat_plus = self.get_end_effector_pose()
                
                # 计算位置偏导数
                jacobian[0:3, i] = (pos_plus - current_pos) / delta
                
                # 计算姿态偏导数 (简化处理)
                jacobian[3:6, i] = (quat_plus[1:4] - current_quat[1:4]) / delta
                
            except Exception:
                pass
        
        # 恢复原始关节角度
        try:
            original_tensor = self.torch.tensor(joint_positions, dtype=self.torch.float32, device=self.device)
            self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(original_tensor))
            self.step_simulation(1)
        except Exception:
            pass
        
        return jacobian
    
    def get_workspace_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        估算机械臂工作空间边界
        """
        # 简化实现：基于关节限位估算
        min_bounds = np.array([-2.0, -2.0, 0.0])  # 默认最小边界
        max_bounds = np.array([2.0, 2.0, 2.0])    # 默认最大边界
        
        return min_bounds, max_bounds
    
    def set_control_mode(self, mode: str) -> bool:
        """
        设置控制模式
        
        Args:
            mode: 控制模式 ("position", "velocity", "torque")
        """
        try:
            from isaacgym import gymapi
            
            if mode == "position":
                drive_mode = gymapi.DOF_MODE_POS
            elif mode == "velocity":
                drive_mode = gymapi.DOF_MODE_VEL
            elif mode == "torque":
                drive_mode = gymapi.DOF_MODE_EFFORT
            else:
                return False
            
            # 获取当前DOF属性
            dof_props = self.gym.get_actor_dof_properties(self.env, self.robot_handle)
            
            # 设置控制模式
            for i in range(len(dof_props)):
                dof_props[i]['driveMode'] = drive_mode
            
            # 应用新的DOF属性
            self.gym.set_actor_dof_properties(self.env, self.robot_handle, dof_props)
            
            return True
            
        except Exception as e:
            print(f"设置控制模式失败: {e}")
            return False
    
    def get_joint_positions(self) -> np.ndarray:
        """获取当前关节位置"""
        self._update_state()
        return self.dof_pos.cpu().numpy()
    
    def get_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        self._update_state()
        return self.dof_vel.cpu().numpy()
    
    def get_joint_torques(self) -> np.ndarray:
        """获取当前关节力矩"""
        self._update_state()
        # 从接触力计算力矩，简化实现
        return self.torch.zeros(self._dof_count).cpu().numpy()
    
    def move_to_joint_positions(self, positions: np.ndarray, 
                               velocity_scale: float = 1.0,
                               acceleration_scale: float = 1.0,
                               blocking: bool = True) -> bool:
        """移动到指定关节位置"""
        if not self.is_ready():
            print("❌ 机械臂未就绪")
            return False
        
        if not self._validate_joint_positions(positions):
            print("❌ 关节位置验证失败")
            return False
        
        try:
            # 检查输入位置是否包含NaN或无穷大值
            if np.any(np.isnan(positions)) or np.any(np.isinf(positions)):
                print(f"❌ 输入位置包含无效值: {positions}")
                return False
            
            # *** 关键优化: 自动限位保护 ***
            safe_positions = self._clamp_to_joint_limits(positions)
            if not np.allclose(positions, safe_positions, atol=1e-6):
                print(f"⚠️  关节位置已自动限制到安全范围")
                print(f"   原始: {positions[:min(4, len(positions))].round(4)}")
                print(f"   修正: {safe_positions[:min(4, len(positions))].round(4)}")
            positions = safe_positions
            
            # 转换为tensor并设置目标
            targets_tensor = self.torch.tensor(positions, dtype=self.torch.float32, device=self.device)
            
            # 使用tensor接口设置目标
            self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
            print(f"✅ 设置关节目标: {positions[:min(6, len(positions))].round(3)}")
            
            if blocking:
                # 等待运动完成
                max_steps = int(10.0 / self.sim_dt)  # 最多等待10秒
                tolerance = 0.02  # 增加容差
                
                for step in range(max_steps):
                    self.step_simulation(1)
                    
                    current_pos = self.get_joint_positions()
                    
                    # 检查当前位置是否包含NaN值
                    if np.any(np.isnan(current_pos)):
                        print(f"❌ 仿真过程中出现NaN值，停止运动")
                        self.stop_motion(emergency=True)
                        return False
                    
                    # 检查当前速度是否过大
                    current_vel = self.get_joint_velocities()
                    if np.any(np.abs(current_vel) > 5.0):  # 如果速度超过5 rad/s
                        print(f"❌ 检测到过大速度 {np.max(np.abs(current_vel)):.3f} rad/s，停止运动")
                        self.stop_motion(emergency=True)
                        return False
                    
                    error = np.abs(current_pos - positions)
                    max_error = np.max(error)
                    
                    # 每100步打印一次进度
                    if step % 100 == 0:
                        print(f"   步骤{step}: 最大误差 {max_error:.4f}")
                    
                    if max_error < tolerance:
                        print(f"✅ 到达目标，步骤: {step}, 误差: {max_error:.4f}")
                        break
                else:
                    print(f"⚠️ 超时，最终误差: {max_error:.4f}")
            
            return True
            
        except Exception as e:
            print(f"❌ 运动控制异常: {e}")
            self._set_state(RobotState.ERROR, f"运动控制失败: {str(e)}")
            return False
    
    def stop_motion(self, emergency: bool = False) -> bool:
        """停止运动"""
        try:
            # 设置当前位置为目标位置
            self._update_state()
            targets_tensor = self.dof_pos.clone()
            self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
            
            if emergency:
                self._set_state(RobotState.EMERGENCY_STOP)
            return True
        except Exception:
            return False
    
    def enable(self) -> bool:
        """使能机械臂"""
        self._enabled = True
        self._set_state(RobotState.IDLE)
        return True
    
    def disable(self) -> bool:
        """失能机械臂"""
        self._enabled = False
        self._set_state(RobotState.DISABLED)
        return True
    
    def is_ready(self) -> bool:
        """检查机械臂是否就绪"""
        return self._enabled and self.state == RobotState.IDLE and self.sim is not None
    
    def is_moving(self) -> bool:
        """检查机械臂是否在运动"""
        current_vel = self.get_joint_velocities()
        return np.any(np.abs(current_vel) > 1e-3)
    
    def home(self) -> bool:
        """回零位"""
        if not self.is_ready():
            return False
        zero_positions = np.zeros(self.dof)
        return self.move_to_joint_positions(zero_positions)
    
    def calibrate(self) -> bool:
        """校准机械臂"""
        return True  # Isaac Gym中默认已校准
    
    def step_simulation(self, steps: int = 1):
        """执行仿真步骤"""
        for _ in range(steps):
            self.gym.simulate(self.sim)
            if self.device == 'cpu':
                self.gym.fetch_results(self.sim, True)
        
        # 刷新tensor
        self._refresh_sim_tensors()
    
    def set_joint_velocities(self, velocities: np.ndarray) -> bool:
        """设置关节速度"""
        if not self._validate_joint_velocities(velocities):
            return False
            
        try:
            vel_tensor = self.torch.tensor(velocities, dtype=self.torch.float32, device=self.device)
            self.gym.set_dof_velocity_target_tensor(self.sim, self.gymtorch.unwrap_tensor(vel_tensor))
            return True
        except Exception:
            return False
    
    def set_joint_torques(self, torques: np.ndarray) -> bool:
        """设置关节力矩"""
        if len(torques) != self.dof:
            return False
            
        try:
            torque_tensor = self.torch.tensor(torques, dtype=self.torch.float32, device=self.device)
            self.gym.set_dof_actuation_force_tensor(self.sim, self.gymtorch.unwrap_tensor(torque_tensor))
            return True
        except Exception:
            return False
    
    # ====== Gym接口兼容 ======
    def reset(self) -> np.ndarray:
        """重置仿真环境"""
        self.home()
        self.step_simulation(10)  # 稳定化
        return self.get_joint_positions()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, dict]:
        """执行动作步骤"""
        success = self.move_to_joint_positions(action, blocking=False)
        self.step_simulation(1)
        
        obs = self.get_joint_positions()
        reward = -np.linalg.norm(obs - action)  # 简单奖励函数
        done = np.allclose(obs, action, atol=1e-2)
        info = {"success": success}
        
        return obs, reward, done, info
    
    def render(self, mode: str = "human", sync_frame_time: bool = True):
        """渲染环境"""
        if mode == "human" and self.viewer:
            # 处理键盘事件
            for evt in self.gym.query_viewer_action_events(self.viewer):
                if evt.action == "QUIT" and evt.value > 0:
                    return False
                elif evt.action == "RESET" and evt.value > 0:
                    self.reset()
                    print("环境已重置")
            
            # 检查窗口是否关闭
            if self.gym.query_viewer_has_closed(self.viewer):
                return False
            
            # 获取结果并绘制
            if self.device != 'cpu':
                self.gym.fetch_results(self.sim, True)
                
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, True)
            
            if sync_frame_time:
                self.gym.sync_frame_time(self.sim)
        elif mode == "rgb_array":
            # 可以实现截图功能
            pass
        
        return True
    
    def close(self):
        """关闭仿真环境"""
        if self.viewer:
            self.gym.destroy_viewer(self.viewer)
        if self.sim:
            self.gym.destroy_sim(self.sim)

 