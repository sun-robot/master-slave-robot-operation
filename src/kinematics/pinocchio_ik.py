"""
基于Pinocchio的逆运动学求解器
使用CasADi优化进行高精度IK求解
"""

import numpy as np
import os
from typing import List, Optional, Tuple, Union
import warnings

# 尝试导入Pinocchio和CasADi，如果没有安装则提供优雅降级
try:
    import pinocchio as pin
    from pinocchio import casadi as cpin
    import casadi
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False


class WeightedMovingFilter:
    """加权移动平均滤波器"""
    
    def __init__(self, weights: np.ndarray, buffer_size: int):
        """
        初始化滤波器
        
        Args:
            weights: 权重数组
            buffer_size: 缓冲区大小
        """
        self.weights = weights / np.sum(weights)  # 归一化权重
        self.buffer_size = buffer_size
        self.buffer = []
        self.filtered_data = None
    
    def add_data(self, data: np.ndarray):
        """添加新数据"""
        self.buffer.append(data.copy())
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)
        
        # 计算加权平均
        if len(self.buffer) >= len(self.weights):
            weighted_sum = np.zeros_like(data)
            for i, weight in enumerate(self.weights):
                weighted_sum += weight * self.buffer[-(i+1)]
            self.filtered_data = weighted_sum
        else:
            # 数据不足时使用简单平均
            self.filtered_data = np.mean(self.buffer, axis=0)


class PinocchioIKSolver:
    """基于Pinocchio的逆运动学求解器"""
    
    def __init__(self, 
                 urdf_path: str,
                 urdf_root_path: str = "",
                 joints_to_lock: Optional[List[str]] = None,
                 end_effector_joint: str = "joint_7",
                 end_effector_link: str = "link_7",
                 end_effector_offset: np.ndarray = np.array([0.0, 0.0, 0.0]),
                 visualization: bool = False):
        """
        初始化Pinocchio IK求解器
        
        Args:
            urdf_path: URDF文件路径
            urdf_root_path: URDF根目录路径
            joints_to_lock: 需要锁定的关节列表
            end_effector_joint: 末端执行器关节名称
            end_effector_link: 末端执行器链接名称
            end_effector_offset: 末端执行器偏移量
            visualization: 是否启用可视化
        """
        if not PINOCCHIO_AVAILABLE:
            raise ImportError("Pinocchio和CasADi是PinocchioIKSolver的必需依赖")
        
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        
        self.visualization = visualization
        self.urdf_path = urdf_path
        self.urdf_root_path = urdf_root_path
        self.end_effector_joint = end_effector_joint
        self.end_effector_link = end_effector_link
        self.end_effector_offset = end_effector_offset
        
        # 加载机器人模型
        self._load_robot_model(joints_to_lock or [])
        
        # 创建CasADi模型
        self._setup_casadi_model()
        
        # 设置优化问题
        self._setup_optimization()
        
        # 初始化滤波器和数据
        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1]), 
            self.reduced_robot.model.nq
        )
        
        # 可视化设置
        self.vis = None
        if self.visualization:
            self._setup_visualization()
    
    def _load_robot_model(self, joints_to_lock: List[str]):
        """加载机器人模型"""
        # 检查URDF文件是否存在
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(f"URDF文件未找到: {self.urdf_path}")
        
        # 加载完整机器人模型
        self.robot = pin.RobotWrapper.BuildFromURDF(self.urdf_path, self.urdf_root_path)
        
        # 创建简化模型（锁定指定关节）
        if joints_to_lock:
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=np.array([0.0] * self.robot.model.nq)
            )
        else:
            self.reduced_robot = self.robot
        
        # 添加末端执行器框架
        try:
            ee_joint_id = self.reduced_robot.model.getJointId(self.end_effector_joint)
            self.reduced_robot.model.addFrame(
                pin.Frame('end_effector',
                         ee_joint_id,
                         pin.SE3(np.eye(3), self.end_effector_offset),
                         pin.FrameType.OP_FRAME)
            )
        except Exception as e:
            print(f"警告: 无法添加末端执行器框架: {e}")
            # 使用默认框架
            self.ee_frame_id = self.reduced_robot.model.getFrameId(self.end_effector_link)
        else:
            self.ee_frame_id = self.reduced_robot.model.getFrameId('end_effector')
    
    def _setup_casadi_model(self):
        """设置CasADi模型"""
        # 创建CasADi模型
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        
        # 创建符号变量
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_target = casadi.SX.sym("tf_target", 4, 4)
        
        # 计算正向运动学
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # 定义误差函数
        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_target],
            [self.cdata.oMf[self.ee_frame_id].translation - self.cTf_target[:3, 3]]
        )
        
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_target],
            [cpin.log3(self.cdata.oMf[self.ee_frame_id].rotation @ self.cTf_target[:3, :3].T)]
        )
    
    def _setup_optimization(self):
        """设置优化问题"""
        # 创建优化器
        self.opti = casadi.Opti()
        
        # 优化变量
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)  # 平滑项
        self.param_tf_target = self.opti.parameter(4, 4)
        
        # 代价函数
        self.translational_cost = casadi.sumsqr(
            self.translational_error(self.var_q, self.param_tf_target)
        )
        self.rotation_cost = casadi.sumsqr(
            self.rotational_error(self.var_q, self.param_tf_target)
        )
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        
        # 总代价函数
        total_cost = (50 * self.translational_cost + 
                     self.rotation_cost + 
                     0.02 * self.regularization_cost + 
                     0.1 * self.smooth_cost)
        
        # 约束条件
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit
        ))
        
        # 设置目标
        self.opti.minimize(total_cost)
        
        # 求解器选项
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-6
            },
            'print_time': False,
            'calc_lam_p': False
        }
        self.opti.solver("ipopt", opts)
    
    def _setup_visualization(self):
        """设置可视化"""
        try:
            from pinocchio.visualize import MeshcatVisualizer
            import meshcat.geometry as mg
            
            # 初始化Meshcat可视化器
            self.vis = MeshcatVisualizer(
                self.reduced_robot.model, 
                self.reduced_robot.collision_model, 
                self.reduced_robot.visual_model
            )
            self.vis.initViewer(open=True)
            self.vis.loadViewerModel("pinocchio")
            self.vis.displayFrames(True, frame_ids=[self.ee_frame_id], 
                                 axis_length=0.15, axis_width=5)
            self.vis.display(pin.neutral(self.reduced_robot.model))
            
            # 添加目标框架可视化
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                         [0, 0, 0], [0, 1, 0],
                         [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                         [0, 1, 0], [0.6, 1, 0],
                         [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            
            axis_length = 0.1
            axis_width = 10
            self.vis.viewer['ee_target'].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )
            
        except ImportError:
            print("警告: Meshcat不可用，跳过可视化设置")
            self.visualization = False
    
    def solve_ik(self, 
                 target_transform: np.ndarray,
                 current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        求解逆运动学
        
        Args:
            target_transform: 4x4目标变换矩阵
            current_joint_angles: 当前关节角度（弧度），可以为None
            
        Returns:
            joint_angles: 求解的关节角度
            success: 是否成功
        """
        # 如果没有提供当前关节角度，使用初始数据
        if current_joint_angles is None:
            current_joint_angles = self.init_data.copy()
        
        # 更新初始猜测
        self.init_data = current_joint_angles.copy()
        
        # 设置初始值
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf_target, target_transform)
        self.opti.set_value(self.var_q_last, self.init_data)
        
        # 可视化目标
        if self.visualization and self.vis is not None:
            self.vis.viewer['ee_target'].set_transform(target_transform)
        
        try:
            # 求解优化问题
            sol = self.opti.solve()
            
            # 获取解
            sol_q = self.opti.value(self.var_q)
            
            # 应用平滑滤波
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data
            
            # 更新初始数据
            self.init_data = sol_q
            
            # 可视化结果
            if self.visualization and self.vis is not None:
                self.vis.display(sol_q)
            
            return sol_q, True
            
        except Exception as e:
            print(f"IK求解失败: {e}")
            
            try:
                # 尝试获取调试解
                sol_q = self.opti.debug.value(self.var_q)
                
                # 应用平滑滤波
                self.smooth_filter.add_data(sol_q)
                sol_q = self.smooth_filter.filtered_data
                
                # 更新初始数据
                self.init_data = sol_q
                
                # 可视化结果
                if self.visualization and self.vis is not None:
                    self.vis.display(sol_q)
                
                return sol_q, False
                
            except:
                # 完全失败，返回当前关节角度
                if current_joint_angles is not None:
                    return current_joint_angles, False
                else:
                    return self.init_data, False
    
    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算正向运动学
        
        Args:
            joint_angles: 关节角度
            
        Returns:
            position: 末端位置
            orientation: 末端姿态（四元数）
        """
        # 更新机器人配置
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, joint_angles)
        
        # 获取末端执行器变换
        transform = self.reduced_robot.data.oMf[self.ee_frame_id]
        
        # 提取位置和旋转
        position = transform.translation
        rotation_matrix = transform.rotation
        
        # 转换为四元数 (w, x, y, z)
        quaternion = pin.Quaternion(rotation_matrix).coeffs()  # [x, y, z, w]
        quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])  # [w, x, y, z]
        
        return position, quaternion
    
    def get_joint_limits(self) -> np.ndarray:
        """获取关节限位"""
        lower = self.reduced_robot.model.lowerPositionLimit
        upper = self.reduced_robot.model.upperPositionLimit
        return np.column_stack([lower, upper])
    
    def get_num_joints(self) -> int:
        """获取关节数量"""
        return self.reduced_robot.model.nq
    
    def set_cost_weights(self, 
                        translation_weight: float = 50.0,
                        rotation_weight: float = 1.0,
                        regularization_weight: float = 0.02,
                        smooth_weight: float = 0.1):
        """
        设置代价函数权重
        
        Args:
            translation_weight: 位置误差权重
            rotation_weight: 姿态误差权重
            regularization_weight: 正则化权重
            smooth_weight: 平滑权重
        """
        total_cost = (translation_weight * self.translational_cost + 
                     rotation_weight * self.rotation_cost + 
                     regularization_weight * self.regularization_cost + 
                     smooth_weight * self.smooth_cost)
        
        self.opti.minimize(total_cost) 