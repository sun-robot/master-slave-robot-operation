#!/usr/bin/env python3
"""
Isaac Gym机械臂仿真接口
基于Isaac Gym的高性能机械臂仿真实现，支持GPU加速和tensor API
"""

import os
import numpy as np
from typing import Optional, Dict, List, Tuple
from scipy.optimize import minimize

# 从主接口文件导入基类和枚举
from .robot_interface import RobotInterface, RobotType, RobotState


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
        asset_options.fix_base_link = True  # 确保基座固定
        asset_options.flip_visual_attachments = False  # 不翻转视觉附件
        asset_options.collapse_fixed_joints = False
        asset_options.disable_gravity = False
        asset_options.thickness = 0.001
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        asset_options.use_mesh_materials = True  # 使用mesh材质
        asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX  # 计算顶点法线
        asset_options.override_com = False  # 不覆盖质心
        asset_options.override_inertia = False  # 不覆盖惯性
        asset_options.vhacd_enabled = False  # 不使用VHACD
        asset_options.density = 1000.0  # 设置密度确保稳定性
        
        print(f"加载URDF: {self.urdf_path}")
        print(f"Asset root: {asset_root}")
        print(f"Asset file: {asset_file}")
        
        # 加载资源
        self.robot_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        
        if self.robot_asset is None:
            raise RuntimeError(f"加载URDF文件失败: {self.urdf_path}")
        
        print(f"✅ URDF资源加载成功: {asset_file}")
        
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
        
        # 检查并修复URDF中的速度限制
        for i in range(self._dof_count):
            if velocity_limits_np[i] <= 0 or np.isnan(velocity_limits_np[i]) or np.isinf(velocity_limits_np[i]):
                velocity_limits_np[i] = 2.0  # 修复无效的速度限制
                print(f"  修复关节{i}速度限制数组: -> 2.0")
        
        print(f"修复后的速度限制: {velocity_limits_np}")
        
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
        start_pose.p = gymapi.Vec3(0.0, 0.0, 0.1)  # 将机械臂提高到地面之上
        start_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        
        self.robot_handle = self.gym.create_actor(
            self.env, self.robot_asset, start_pose, 
            f"robot_{self.robot_id}", 0, 0
        )
        
        if self.robot_handle is None:
            raise RuntimeError("创建机械臂actor失败")
        
        print(f"✅ 机械臂actor创建成功，位置: ({start_pose.p.x}, {start_pose.p.y}, {start_pose.p.z})")
        
        # 获取URDF中的DOF属性，但修复明显错误的值
        dof_props = self.gym.get_actor_dof_properties(self.env, self.robot_handle)
        print("检查并修复URDF中的DOF属性...")
        
        # 修复URDF中明显错误的速度和力矩限制，并设置稳定参数
        for i in range(self._dof_count):
            # 如果速度限制为0，设置为合理的默认值
            if dof_props['velocity'][i] <= 0:
                dof_props['velocity'][i] = 2.0  # 默认2.0 rad/s
                print(f"  修复关节{i}速度限制: 0 -> 2.0")
            
            # 如果力矩限制为0，设置为合理的默认值
            if dof_props['effort'][i] <= 0:
                dof_props['effort'][i] = 100.0  # 默认100 N·m
                print(f"  修复关节{i}力矩限制: 0 -> 100.0")
            
            # 设置足够的刚度和阻尼来保持机械臂稳定
            dof_props['stiffness'][i] = 1000.0  # 高刚度保持位置
            dof_props['damping'][i] = 50.0      # 适当阻尼减少振荡
            dof_props['driveMode'][i] = gymapi.DOF_MODE_POS  # 位置控制模式
            
            print(f"  关节{i}: 刚度=1000.0, 阻尼=50.0")
        
        # 应用修复后的DOF属性
        self.gym.set_actor_dof_properties(self.env, self.robot_handle, dof_props)
        
        # *** 最基本的初始化：设置零位，防止NaN ***
        print("设置基本的零位初始化...")
        zero_positions = np.zeros(self._dof_count, dtype=np.float32)
        try:
            self.gym.set_actor_dof_position_targets(
                self.env, self.robot_handle, zero_positions
            )
            print(f"✅ 设置初始位置为零位: {zero_positions}")
        except Exception as e:
            print(f"⚠️ 初始位置设置失败: {e}")
    
    def _prepare_sim_tensors(self):
        """准备仿真tensor（使用URDF原始配置）"""
        # 准备仿真（使用URDF默认配置）
        print("准备仿真环境...")
        self.gym.prepare_sim(self.sim)
        
        # *** 基本修复：运行几步仿真让状态稳定，防止NaN ***
        print("运行基础仿真步骤以初始化状态...")
        
        # 设置零位目标来保持机械臂稳定
        zero_positions = np.zeros(self._dof_count, dtype=np.float32)
        
        for i in range(20):  # 增加到20步以确保稳定
            # 每步都设置位置目标，确保机械臂保持零位
            try:
                self.gym.set_actor_dof_position_targets(
                    self.env, self.robot_handle, zero_positions
                )
            except:
                pass  # 如果设置失败就跳过
                
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)
            
            if i % 5 == 0:
                print(f"  稳定化步骤: {i}/20")
        
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
        # DOF状态tensor格式：[[pos1, vel1], [pos2, vel2], ...] 每行是位置和速度对
        self.dof_pos = self.dof_state[:, 0]  # 第0列是位置
        self.dof_vel = self.dof_state[:, 1]  # 第1列是速度
        
        # 重塑刚体状态 (每个刚体13个状态：pos(3) + quat(4) + vel(3) + ang_vel(3))
        self.rigid_body_pos = self.rigid_body_states.view(-1, 13)[:self._body_count, 0:3]
        self.rigid_body_rot = self.rigid_body_states.view(-1, 13)[:self._body_count, 3:7]
        self.rigid_body_vel = self.rigid_body_states.view(-1, 13)[:self._body_count, 7:10]
        self.rigid_body_ang_vel = self.rigid_body_states.view(-1, 13)[:self._body_count, 10:13]
        
        print(f"✅ 仿真tensor准备完成")
        print(f"   DOF状态: {self.dof_state.shape}")
        print(f"   刚体状态: {self.rigid_body_states.shape}")
    
    
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
    
    # ===================== 基本属性实现 =====================
    
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
    
    # ===================== 运动学方法实现 =====================
    
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
        """获取末端执行器位姿"""
        try:
            self._update_state()
            
            # 假设最后一个刚体是末端执行器
            ee_pos = self.rigid_body_pos[-1].cpu().numpy()
            ee_quat = self.rigid_body_rot[-1].cpu().numpy()
            
            return ee_pos, ee_quat
            
        except Exception as e:
            print(f"获取末端执行器位姿失败: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])
    
    def get_jacobian(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """获取雅可比矩阵"""
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
    
    def solve_ik(self, target_pose: np.ndarray, current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """使用数值方法求解逆运动学"""
        try:
            from isaacgym import gymapi
            
            # 如果没有提供当前关节角度，使用当前位置
            if current_joint_angles is None:
                current_joint_angles = self.get_joint_positions()
            
            # 从4x4位姿矩阵提取位置和姿态
            target_position = target_pose[:3, 3]
            target_orientation = self._rotation_matrix_to_quaternion(target_pose[:3, :3])
            
            def objective(q):
                # 设置关节角度并获取末端位姿（使用tensor API）
                targets_tensor = self.torch.tensor(q.astype(np.float32), dtype=self.torch.float32, device=self.device)
                self.gym.set_dof_position_target_tensor(self.sim, self.gymtorch.unwrap_tensor(targets_tensor))
                self.step_simulation(1)
                
                current_pos, current_quat = self.get_end_effector_pose()
                
                # 位置误差
                pos_error = np.linalg.norm(current_pos - target_position)
                
                # 姿态误差 (简化为四元数差的模长)
                quat_error = np.linalg.norm(current_quat - target_orientation)
                
                return pos_error + quat_error
            
            # 关节限位约束
            bounds = [(limits[0], limits[1]) for limits in self.joint_limits]
            
            # 优化求解
            result = minimize(
                objective, current_joint_angles, 
                method='L-BFGS-B', bounds=bounds,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            success = result.success and result.fun < 1e-3
            return result.x, success
            
        except Exception as e:
            print(f"IK求解失败: {e}")
            if current_joint_angles is not None:
                return current_joint_angles, False
            else:
                return np.zeros(self.dof), False
    
    # ===================== 状态获取方法实现 =====================
    
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
    
    # ===================== 控制方法实现 =====================
    
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
    
    def move_to_pose(self, position: np.ndarray, orientation: np.ndarray,
                    velocity_scale: float = 1.0,
                    acceleration_scale: float = 1.0,
                    blocking: bool = True) -> bool:
        """移动到指定末端位姿"""
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
        """直线运动到目标位姿"""
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
    
    def set_control_mode(self, mode: str) -> bool:
        """设置控制模式"""
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
    
    # ===================== 机械臂状态管理 =====================
    
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
    
    # ===================== 仿真特有方法 =====================
    
    def step_simulation(self, steps: int = 1):
        """执行仿真步骤"""
        for _ in range(steps):
            self.gym.simulate(self.sim)
            if self.device == 'cpu':
                self.gym.fetch_results(self.sim, True)
        
        # 刷新tensor
        self._refresh_sim_tensors()
    
    def get_workspace_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """估算机械臂工作空间边界"""
        # 简化实现：基于关节限位估算
        min_bounds = np.array([-2.0, -2.0, 0.0])  # 默认最小边界
        max_bounds = np.array([2.0, 2.0, 2.0])    # 默认最大边界
        
        return min_bounds, max_bounds
    
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


# 便捷创建函数
def create_simulated_robot(robot_type: RobotType, urdf_path: str, **kwargs) -> SimulatedRobotInterface:
    """便捷函数：创建仿真机械臂接口"""
    return SimulatedRobotInterface(robot_type, urdf_path, **kwargs) 