#!/usr/bin/env python3
"""
MuJoCo机械臂仿真接口
基于MuJoCo的高性能机械臂仿真实现，支持XML模型文件和Python API
"""

import os
import numpy as np
from typing import Optional, Dict, List, Tuple
from scipy.optimize import minimize

# 从主接口文件导入基类和枚举
from .robot_interface import RobotInterface, RobotType, RobotState


class MuJoCoRobotInterface(RobotInterface):
    """
    基于MuJoCo的机械臂仿真接口
    使用XML模型文件创建仿真环境，支持高性能物理仿真
    """
    
    def __init__(self, robot_type: RobotType, xml_path: str, 
                 robot_id: str = "mujoco_robot", headless: bool = False,
                 sim_params: Optional[Dict] = None, render_mode: str = "human"):
        """
        初始化MuJoCo仿真环境
        
        Args:
            robot_type: 机械臂类型
            xml_path: XML模型文件路径
            robot_id: 机械臂唯一标识符
            headless: 是否无界面模式
            sim_params: 仿真参数字典
            render_mode: 渲染模式 ("human", "rgb_array", "depth")
        """
        super().__init__(robot_type, robot_id)
        
        try:
            import mujoco
            from mujoco import MjData, MjModel
            # 导入MuJoCo可视化模块
            import mujoco.viewer
        except ImportError:
            raise ImportError("MuJoCo未安装，请按照MuJoCo文档安装: pip install mujoco")
        
        self.mujoco = mujoco
        self.MjData = MjData
        self.MjModel = MjModel
        self.xml_path = xml_path
        self.headless = headless
        self.render_mode = render_mode
        
        # 仿真参数
        self.sim_params = self._parse_sim_params(sim_params)
        self.sim_dt = self.sim_params.get('dt', 0.002)  # 默认500Hz
            
        # MuJoCo对象
        self.model = None
        self.data = None
        self.viewer = None
        self.viewer_context = None
        
        # 机械臂属性
        self._enabled = False
        self._dof_count = 0
        self._joint_names = []
        self._body_names = []
        self._joint_limits = None
        self._velocity_limits = None
        self._torque_limits = None
        
        # 初始化仿真环境
        self._load_model()
        self._setup_viewer()
        self._initialize_robot_properties()
        
        print(f"✅ MuJoCo仿真环境初始化完成")
        print(f"   机械臂: {self.name}")
        print(f"   自由度: {self.dof}")
        print(f"   模型文件: {os.path.basename(self.xml_path)}")
        
    def _parse_sim_params(self, custom_params: Optional[Dict] = None) -> Dict:
        """解析仿真参数"""
        default_params = {
            'dt': 0.002,  # 仿真时间步长 (500Hz)
            'substeps': 1,  # 子步数
            'gravity': [0, 0, -9.81],  # 重力
            'wind': [0, 0, 0],  # 风力
            'magnetic': [0, 0, 0],  # 磁场
            'density': 1.0,  # 空气密度
            'viscosity': 0.0,  # 空气粘度
            'integrator': 'RK4',  # 积分器类型
            'collision': 'all',  # 碰撞检测
            'cone': 'pyramidal',  # 摩擦锥类型
            'jacobian': 'dense',  # 雅可比矩阵类型
            'solver': 'Newton',  # 求解器类型
            'iterations': 100,  # 最大迭代次数
            'tolerance': 1e-10,  # 收敛容差
            'noslip_iterations': 0,  # 无滑移迭代
            'mpr_iterations': 50,  # MPR迭代
            'max_contact_points': 8,  # 最大接触点数
        }
        
        # 应用自定义参数
        if custom_params:
            default_params.update(custom_params)
            
        return default_params
    
    def _load_model(self):
        """加载MuJoCo模型"""
        if not os.path.exists(self.xml_path):
            raise FileNotFoundError(f"XML模型文件不存在: {self.xml_path}")
        
        print(f"加载MuJoCo模型: {self.xml_path}")
        
        try:
            # 加载模型
            self.model = self.MjModel.from_xml_path(self.xml_path)
            self.data = self.MjData(self.model)
            
            print(f"✅ MuJoCo模型加载成功")
            print(f"   自由度: {self.model.nq}")
            print(f"   执行器数: {self.model.nu}")
            print(f"   刚体数: {self.model.nbody}")
            print(f"   几何体数: {self.model.ngeom}")
            
        except Exception as e:
            raise RuntimeError(f"加载MuJoCo模型失败: {e}")
    
    def _initialize_robot_properties(self):
        """初始化机械臂属性"""
        # 获取自由度数量
        self._dof_count = self.model.nq
        
        # 获取关节名称
        self._joint_names = []
        for i in range(self.model.njnt):
            joint_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                self._joint_names.append(joint_name)
        
        # 获取刚体名称
        self._body_names = []
        for i in range(self.model.nbody):
            body_name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name:
                self._body_names.append(body_name)
        
        # 获取关节限位
        joint_limits = np.zeros((self._dof_count, 2))
        velocity_limits = np.zeros(self._dof_count)
        torque_limits = np.zeros(self._dof_count)
        
        for i in range(self._dof_count):
            if i < self.model.njnt:
                # 关节限位
                joint_limits[i, 0] = self.model.jnt_range[i, 0] if self.model.jnt_range[i, 0] != 0 else -np.pi
                joint_limits[i, 1] = self.model.jnt_range[i, 1] if self.model.jnt_range[i, 1] != 0 else np.pi
                
                # 速度限制 (从执行器获取或使用默认值)
                if i < self.model.nu:
                    velocity_limits[i] = self.model.actuator_ctrlrange[i, 1] if self.model.actuator_ctrlrange[i, 1] != 0 else 2.0
                else:
                    velocity_limits[i] = 2.0  # 默认2 rad/s
                
                # 力矩限制
                if i < self.model.nu:
                    torque_limits[i] = self.model.actuator_forcerange[i, 1] if self.model.actuator_forcerange[i, 1] != 0 else 100.0
                else:
                    torque_limits[i] = 100.0  # 默认100 N·m
        
        self._joint_limits = joint_limits
        self._velocity_limits = velocity_limits
        self._torque_limits = torque_limits
        
        print(f"✅ 机械臂属性初始化完成")
        print(f"   关节名称: {self._joint_names}")
        print(f"   关节限位: {joint_limits}")
        print(f"   速度限制: {velocity_limits}")
    
    def _setup_viewer(self):
        """设置可视化窗口"""
        if not self.headless and self.render_mode == "human":
            try:
                # 使用MuJoCo viewer启动被动模式
                self.viewer_context = self.mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer = self.viewer_context.__enter__()
                print(f"✅ MuJoCo可视化窗口创建成功")
            except Exception as e:
                print(f"⚠️ 创建可视化窗口失败: {e}")
                self.viewer = None
                self.viewer_context = None
        else:
            self.viewer = None
            self.viewer_context = None
    
    def _clamp_to_joint_limits(self, positions: np.ndarray) -> np.ndarray:
        """将关节位置限制到安全范围内"""
        if len(positions) != self._dof_count:
            print(f"❌ 位置数组长度({len(positions)}) 与自由度({self._dof_count})不匹配")
            return positions
        
        clamped_positions = positions.copy()
        
        for i in range(len(positions)):
            lower_limit = self._joint_limits[i, 0]
            upper_limit = self._joint_limits[i, 1]
            original_pos = positions[i]
            
            # 限制到安全范围
            clamped_pos = np.clip(positions[i], lower_limit, upper_limit)
            clamped_positions[i] = clamped_pos
            
            # 报告显著的修正
            if abs(original_pos - clamped_pos) > 1e-4:
                print(f"   关节{i+1}: {original_pos:.4f} → {clamped_pos:.4f} "
                      f"(限位: [{lower_limit:.3f}, {upper_limit:.3f}])")
        
        return clamped_positions
    
    def _update_state(self):
        """更新机械臂状态"""
        if not self.data:
            return
        
        # MuJoCo会自动更新状态，无需手动刷新
    
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
        return self._joint_limits
    
    @property
    def velocity_limits(self) -> np.ndarray:
        """关节速度限位"""
        return self._velocity_limits
    
    @property
    def acceleration_limits(self) -> np.ndarray:
        """关节加速度限位"""
        # MuJoCo中没有直接的加速度限位，返回默认值
        return np.ones(self.dof) * 10.0
    
    @property
    def name(self) -> str:
        """机械臂名称"""
        return f"MuJoCo {self.robot_type.value}"
    
    # ===================== 运动学方法实现 =====================
    
    def get_dh_params(self) -> np.ndarray:
        """
        获取DH参数 - MuJoCo中从XML推导
        注意：这是一个简化实现，实际应该从XML解析
        """
        # 返回默认DH参数矩阵，实际应该从XML文件解析
        return np.zeros((self.dof, 4))
    
    def get_base_transform(self) -> np.ndarray:
        """获取基座变换矩阵"""
        if self.data:
            # 获取基座刚体的变换矩阵
            base_id = 0  # 假设基座是第一个刚体
            if base_id < self.model.nbody:
                pos = self.data.xpos[base_id]
                quat = self.data.xquat[base_id]
                
                # 构建4x4变换矩阵
                transform = np.eye(4)
                transform[:3, 3] = pos
                
                # 四元数转旋转矩阵
                from scipy.spatial.transform import Rotation as R
                rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
                transform[:3, :3] = rot.as_matrix()
                
                return transform
        
        return np.eye(4)
    
    def get_end_effector_transform(self) -> np.ndarray:
        """获取末端执行器相对于最后一个关节的变换"""
        if self.data and self.model.nbody > 1:
            # 获取最后一个刚体的变换
            ee_id = self.model.nbody - 1
            pos = self.data.xpos[ee_id]
            quat = self.data.xquat[ee_id]
            
            # 构建4x4变换矩阵
            transform = np.eye(4)
            transform[:3, 3] = pos
            
            # 四元数转旋转矩阵
            from scipy.spatial.transform import Rotation as R
            rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            transform[:3, :3] = rot.as_matrix()
            
            return transform
        
        return np.eye(4)
    
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位姿"""
        try:
            if self.data and self.model.nbody > 1:
                # 获取最后一个刚体的位置和姿态
                ee_id = self.model.nbody - 1
                pos = self.data.xpos[ee_id].copy()
                quat = self.data.xquat[ee_id].copy()
                
                return pos, quat
            else:
                return np.zeros(3), np.array([1, 0, 0, 0])
                
        except Exception as e:
            print(f"获取末端执行器位姿失败: {e}")
            return np.zeros(3), np.array([1, 0, 0, 0])
    
    def get_jacobian(self, joint_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """获取雅可比矩阵"""
        try:
            if joint_positions is not None:
                # 设置关节位置
                self.data.qpos[:self.dof] = joint_positions
                self.mujoco.mj_forward(self.model, self.data)
            
            # 获取雅可比矩阵
            jacobian = np.zeros((6, self.dof))
            
            # 获取末端执行器的雅可比矩阵
            if self.model.nbody > 1:
                ee_id = self.model.nbody - 1
                self.mujoco.mj_jac(self.model, self.data, jacobian[:3, :], jacobian[3:, :], 
                                  np.zeros(3), ee_id)
            
            return jacobian
                
        except Exception as e:
            print(f"雅可比矩阵计算失败: {e}")
            return np.zeros((6, self.dof))
    
    def solve_ik(self, target_pose: np.ndarray, current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """使用数值方法求解逆运动学"""
        try:
            # 如果没有提供当前关节角度，使用当前位置
            if current_joint_angles is None:
                current_joint_angles = self.get_joint_positions()
            
            # 从4x4位姿矩阵提取位置和姿态
            target_position = target_pose[:3, 3]
            from scipy.spatial.transform import Rotation as R
            target_orientation = R.from_matrix(target_pose[:3, :3]).as_quat()
            target_quat = np.array([target_orientation[3], target_orientation[0], 
                                   target_orientation[1], target_orientation[2]])
            
            def objective(q):
                # 设置关节角度并获取末端位姿
                self.data.qpos[:self.dof] = q
                self.mujoco.mj_forward(self.model, self.data)
                
                current_pos, current_quat = self.get_end_effector_pose()
                
                # 位置误差
                pos_error = np.linalg.norm(current_pos - target_position)
                
                # 姿态误差 (简化为四元数差的模长)
                quat_error = np.linalg.norm(current_quat - target_quat)
                
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
        if self.data:
            return self.data.qpos[:self.dof].copy()
        return np.zeros(self.dof)
    
    def get_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        if self.data:
            return self.data.qvel[:self.dof].copy()
        return np.zeros(self.dof)
    
    def get_joint_torques(self) -> np.ndarray:
        """获取当前关节力矩"""
        if self.data:
            return self.data.qfrc_applied[:self.dof].copy()
        return np.zeros(self.dof)
    
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
            
            # 自动限位保护
            safe_positions = self._clamp_to_joint_limits(positions)
            if not np.allclose(positions, safe_positions, atol=1e-6):
                print(f"⚠️  关节位置已自动限制到安全范围")
                print(f"   原始: {positions[:min(4, len(positions))].round(4)}")
                print(f"   修正: {safe_positions[:min(4, len(positions))].round(4)}")
            positions = safe_positions
            
            # 使用PD控制器实现平滑运动
            print(f"✅ 设置关节目标: {positions[:min(8, len(positions))].round(3)}")
            
            if blocking:
                # 等待运动完成
                max_steps = int(10.0 / self.sim_dt)  # 最多等待10秒
                tolerance = 0.02  # 增加容差
                
                # PD控制器参数 - 使用更保守的增益以获得更稳定的控制
                kp = 2.0   # 位置增益（降低）
                kd = 1.0   # 速度增益（降低）
                
                for step in range(max_steps):
                    current_pos = self.get_joint_positions()
                    current_vel = self.get_joint_velocities()
                    
                    # 计算误差
                    pos_error = positions - current_pos
                    vel_error = np.zeros_like(current_vel) - current_vel  # 目标速度为0
                    
                    # PD控制律
                    control_signal = kp * pos_error + kd * vel_error
                    
                    # 限制控制信号 - 使用更保守的最大控制信号
                    max_control = 1.0  # 最大控制信号（降低）
                    control_signal = np.clip(control_signal, -max_control, max_control)
                    
                    # 应用控制信号 - 只控制有执行器的关节
                    num_actuators = len(self.data.ctrl)
                    if len(control_signal) > num_actuators:
                        # 如果控制信号长度超过执行器数量，只取前num_actuators个
                        self.data.ctrl[:] = control_signal[:num_actuators]
                    else:
                        # 否则直接应用
                        self.data.ctrl[:len(control_signal)] = control_signal
                    
                    # 执行仿真步骤
                    self.step_simulation(1)
                    
                    # 检查当前位置是否包含NaN值
                    if np.any(np.isnan(current_pos)):
                        print(f"❌ 仿真过程中出现NaN值，停止运动")
                        self.stop_motion(emergency=True)
                        return False
                    
                    # 检查当前速度是否过大
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
                        self._set_state(RobotState.IDLE)  # 重置状态
                        break
                else:
                    print(f"⚠️ 超时，最终误差: {max_error:.4f}")
                    self._set_state(RobotState.IDLE)  # 重置状态
            
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
            num_steps = int(np.linalg.norm(target_position - current_pos) / (velocity * self.sim_dt))
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
            current_pos = self.get_joint_positions()
            self.data.ctrl[:self.dof] = current_pos
            
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
            # 在MuJoCo中，通过设置控制信号来实现速度控制
            # 这里简化实现，实际需要更复杂的控制器
            self.data.ctrl[:self.dof] = velocities
            return True
        except Exception:
            return False
    
    def set_joint_torques(self, torques: np.ndarray) -> bool:
        """设置关节力矩"""
        if len(torques) != self.dof:
            return False
            
        try:
            # 在MuJoCo中，通过设置控制信号来实现力矩控制
            self.data.ctrl[:self.dof] = torques
            return True
        except Exception:
            return False
    
    def set_control_mode(self, mode: str) -> bool:
        """设置控制模式"""
        # MuJoCo中控制模式通常在XML中定义
        # 这里提供接口兼容性
        print(f"控制模式设置为: {mode}")
        return True
    
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
        # 在MuJoCo中，只要启用了且数据有效就认为就绪
        return self._enabled and self.data is not None
    
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
        return True  # MuJoCo中默认已校准
    
    # ===================== 仿真特有方法 =====================
    
    def step_simulation(self, steps: int = 1):
        """执行仿真步骤"""
        for _ in range(steps):
            self.mujoco.mj_step(self.model, self.data)
    
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
            try:
                if hasattr(self.viewer, 'is_running') and self.viewer.is_running():
                    self.viewer.sync()
                    return True
                else:
                    return False
            except Exception as e:
                print(f"渲染失败: {e}")
                return False
        elif mode == "rgb_array":
            # 可以实现截图功能
            pass
        
        return True
    
    def close(self):
        """关闭仿真环境"""
        if self.viewer_context:
            try:
                self.viewer_context.__exit__(None, None, None)
            except:
                pass
        if self.viewer:
            try:
                self.viewer.close()
            except:
                pass
        if self.data:
            del self.data
        if self.model:
            del self.model


# 便捷创建函数
def create_mujoco_robot(robot_type: RobotType, xml_path: str, **kwargs) -> MuJoCoRobotInterface:
    """便捷函数：创建MuJoCo仿真机械臂接口"""
    return MuJoCoRobotInterface(robot_type, xml_path, **kwargs)
  