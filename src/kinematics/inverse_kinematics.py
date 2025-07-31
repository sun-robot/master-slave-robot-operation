"""
逆向运动学模块
实现末端位姿到关节角度的转换
"""

import numpy as np
from typing import List, Tuple, Optional
import transforms3d as t3d
from scipy.optimize import minimize
from .forward_kinematics import ForwardKinematics

# 尝试导入Pinocchio IK求解器
try:
    from .pinocchio_ik import PinocchioIKSolver
    PINOCCHIO_IK_AVAILABLE = True
except ImportError:
    PINOCCHIO_IK_AVAILABLE = False


class InverseKinematics:
    """逆向运动学计算类"""
    
    def __init__(self, fk_solver: ForwardKinematics, joint_limits: Optional[np.ndarray] = None,
                 urdf_path: Optional[str] = None, urdf_root_path: str = "",
                 joints_to_lock: Optional[List[str]] = None,
                 end_effector_joint: str = "joint_7",
                 end_effector_link: str = "link_7",
                 end_effector_offset: np.ndarray = np.array([0.0, 0.0, 0.0]),
                 use_pinocchio: bool = False):
        """
        初始化逆向运动学
        
        Args:
            fk_solver: 正向运动学求解器
            joint_limits: 关节限位，形状为(n_joints, 2)，每行包含[min, max]
            urdf_path: URDF文件路径（用于Pinocchio）
            urdf_root_path: URDF根目录路径
            joints_to_lock: 需要锁定的关节列表
            end_effector_joint: 末端执行器关节名称
            end_effector_link: 末端执行器链接名称
            end_effector_offset: 末端执行器偏移量
            use_pinocchio: 是否使用Pinocchio IK求解器
        """
        self.fk_solver = fk_solver
        self.joint_limits = joint_limits
        self.n_joints = fk_solver.n_joints
        self.use_pinocchio = use_pinocchio and PINOCCHIO_IK_AVAILABLE
        
        # 初始化Pinocchio IK求解器（如果可用且被请求）
        self.pinocchio_ik = None
        if self.use_pinocchio and urdf_path:
            try:
                self.pinocchio_ik = PinocchioIKSolver(
                    urdf_path=urdf_path,
                    urdf_root_path=urdf_root_path,
                    joints_to_lock=joints_to_lock,
                    end_effector_joint=end_effector_joint,
                    end_effector_link=end_effector_link,
                    end_effector_offset=end_effector_offset,
                    visualization=False
                )
                print("Pinocchio IK求解器已初始化")
            except Exception as e:
                print(f"Pinocchio IK求解器初始化失败: {e}")
                self.use_pinocchio = False
        
    def ik_jacobian_method(self, target_position: np.ndarray, target_orientation: np.ndarray,
                          initial_guess: Optional[np.ndarray] = None, max_iterations: int = 100,
                          tolerance: float = 1e-3) -> Tuple[np.ndarray, bool]:
        """
        使用雅可比矩阵方法求解逆向运动学
        
        Args:
            target_position: 目标位置 (x, y, z)
            target_orientation: 目标姿态 (四元数)
            initial_guess: 初始关节角度猜测，可以为None
            max_iterations: 最大迭代次数
            tolerance: 收敛容差
            
        Returns:
            joint_angles: 求解的关节角度
            success: 是否成功收敛
        """
        # 如果没有提供初始猜测，使用零位
        if initial_guess is None:
            initial_guess = np.zeros(self.n_joints)
        
        joint_angles = initial_guess.copy()
        
        for iteration in range(max_iterations):
            # 计算当前位姿
            current_position, current_orientation = self.fk_solver.forward_kinematics(joint_angles)
            
            # 计算位置和姿态误差
            position_error = target_position - current_position
            orientation_error = self._quaternion_error(target_orientation, current_orientation)
            
            # 组合误差向量
            error = np.concatenate([position_error, orientation_error])
            
            # 检查收敛
            if np.linalg.norm(error) < tolerance:
                return joint_angles, True
            
            # 计算雅可比矩阵
            jacobian = self.fk_solver.jacobian(joint_angles)
            
            # 使用伪逆求解关节角度增量
            try:
                jacobian_pinv = np.linalg.pinv(jacobian)
                delta_theta = jacobian_pinv @ error
                
                # 更新关节角度
                joint_angles += delta_theta
            except np.linalg.LinAlgError:
                # 雅可比矩阵奇异，无法求解
                return joint_angles, False
        
        return joint_angles, False
    
    def ik_optimization_method(self, target_position: np.ndarray, target_orientation: np.ndarray,
                             initial_guess: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        使用优化方法求解逆向运动学
        
        Args:
            target_position: 目标位置
            target_orientation: 目标姿态
            initial_guess: 初始猜测，可以为None
            
        Returns:
            joint_angles: 求解的关节角度
            success: 是否成功
        """
        # 如果没有提供初始猜测，使用零位
        if initial_guess is None:
            initial_guess = np.zeros(self.n_joints)
        
        def objective_function(joint_angles):
            """目标函数：位姿误差"""
            current_position, current_orientation = self.fk_solver.forward_kinematics(joint_angles)
            
            # 位置误差
            position_error = np.linalg.norm(target_position - current_position)
            
            # 姿态误差
            orientation_error = np.linalg.norm(self._quaternion_error(target_orientation, current_orientation))
            
            return position_error + orientation_error
        
        # 设置约束条件（关节限位）
        bounds = None
        if self.joint_limits is not None:
            bounds = [(self.joint_limits[i, 0], self.joint_limits[i, 1]) 
                     for i in range(self.n_joints)]
        
        # 优化求解
        result = minimize(objective_function, initial_guess, 
                        method='L-BFGS-B', bounds=bounds,
                        options={'maxiter': 1000})
        
        return result.x, result.success
    
    def ik_pinocchio_method(self, target_position: np.ndarray, target_orientation: np.ndarray,
                           initial_guess: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        使用Pinocchio求解逆向运动学
        
        Args:
            target_position: 目标位置
            target_orientation: 目标姿态
            initial_guess: 初始猜测，可以为None
            
        Returns:
            joint_angles: 求解的关节角度
            success: 是否成功
        """
        if not self.use_pinocchio or self.pinocchio_ik is None:
            print("Pinocchio IK求解器不可用，回退到优化方法")
            return self.ik_optimization_method(target_position, target_orientation, initial_guess)
        
        # 构造目标变换矩阵
        target_transform = np.eye(4)
        target_transform[:3, 3] = target_position
        target_transform[:3, :3] = t3d.quaternions.quat2mat(target_orientation)
        
        # 使用Pinocchio求解IK
        try:
            joint_angles, success = self.pinocchio_ik.solve_ik(
                target_transform, 
                initial_guess
            )
            return joint_angles, success
        except Exception as e:
            print(f"Pinocchio IK求解失败: {e}")
            return self.ik_optimization_method(target_position, target_orientation, initial_guess)
    
    def _quaternion_error(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        计算四元数误差
        
        Args:
            q1: 目标四元数
            q2: 当前四元数
            
        Returns:
            四元数误差
        """
        # 确保四元数为单位四元数
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 计算四元数差
        error = q1 - q2
        return error
    
    def multiple_solutions(self, target_position: np.ndarray, target_orientation: np.ndarray,
                         num_solutions: int = 8) -> List[np.ndarray]:
        """
        寻找多个IK解
        
        Args:
            target_position: 目标位置
            target_orientation: 目标姿态
            num_solutions: 期望的解的数量
            
        Returns:
            多个关节角度解的列表
        """
        solutions = []
        
        # 生成多个初始猜测
        if self.joint_limits is not None:
            min_angles = self.joint_limits[:, 0]
            max_angles = self.joint_limits[:, 1]
        else:
            min_angles = -np.pi * np.ones(self.n_joints)
            max_angles = np.pi * np.ones(self.n_joints)
        
        for i in range(num_solutions):
            # 随机生成初始猜测
            initial_guess = np.random.uniform(min_angles, max_angles)
            
            # 求解IK
            joint_angles, success = self.ik_optimization_method(
                target_position, target_orientation, initial_guess)
            
            if success:
                # 检查是否与已有解重复
                is_duplicate = False
                for existing_solution in solutions:
                    if np.linalg.norm(joint_angles - existing_solution) < 0.1:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    solutions.append(joint_angles)
        
        return solutions
    
    def select_best_solution(self, solutions: List[np.ndarray], 
                           current_angles: np.ndarray) -> np.ndarray:
        """
        从多个解中选择最佳解
        
        Args:
            solutions: 多个关节角度解
            current_angles: 当前关节角度
            
        Returns:
            最佳关节角度解
        """
        if not solutions:
            return current_angles
        
        # 选择与当前角度最接近的解
        best_solution = solutions[0]
        min_distance = np.linalg.norm(solutions[0] - current_angles)
        
        for solution in solutions[1:]:
            distance = np.linalg.norm(solution - current_angles)
            if distance < min_distance:
                min_distance = distance
                best_solution = solution
        
        return best_solution 