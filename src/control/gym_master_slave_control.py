"""
基于Gym仿真的主从机械臂控制系统
主机械臂：Gym仿真机械臂
从机械臂：UR3e机械臂（也可以是gym仿真）
"""

import numpy as np
import time
import threading
import logging
from typing import Dict, Optional, Tuple, Union
from ..robot_models.robot_factory import RobotFactory
from ..robot_models.robot_interface import RobotType, RobotState
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics
from ..utils.math_utils import normalize_quaternion


class GymMasterSlaveController:
    """
    基于Gym仿真的主从机械臂控制器
    """
    
    def __init__(self, 
                 master_type: RobotType = RobotType.UNITREE_D1,
                 slave_type: RobotType = RobotType.UR3E,
                 master_render: bool = True,
                 slave_render: bool = False,
                 control_frequency: float = 50.0):
        """
        初始化Gym主从控制器
        
        Args:
            master_type: 主机械臂类型
            slave_type: 从机械臂类型
            master_render: 主机械臂是否渲染
            slave_render: 从机械臂是否渲染
            control_frequency: 控制频率
        """
        # 配置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # 控制参数
        self.control_frequency = control_frequency
        self.control_dt = 1.0 / control_frequency
        
        # 创建主机械臂（Gym仿真）
        try:
            self.master_robot = RobotFactory.create_gym_robot(
                robot_type=master_type,
                robot_id="gym_master",
                render_mode="human" if master_render else None,
                control_frequency=control_frequency
            )
            self.logger.info(f"创建主机械臂: {self.master_robot.name}")
        except Exception as e:
            self.logger.error(f"创建主机械臂失败: {e}")
            raise
        
        # 创建从机械臂（也使用Gym仿真）
        try:
            self.slave_robot = RobotFactory.create_gym_robot(
                robot_type=slave_type,
                robot_id="gym_slave",
                render_mode="human" if slave_render else None,
                control_frequency=control_frequency
            )
            self.logger.info(f"创建从机械臂: {self.slave_robot.name}")
        except Exception as e:
            self.logger.error(f"创建从机械臂失败: {e}")
            raise
        
        # 状态管理
        self.is_running = False
        self.is_connected = False
        self.emergency_stop_flag = False
        
        # 主从变换矩阵（从机械臂相对主机械臂的位置）
        self.master_to_slave_transform = np.eye(4)
        self.master_to_slave_transform[0, 3] = 1.0  # X方向偏移1米
        
        # 运动学求解器
        self._setup_kinematics()
        
        # 控制线程
        self.control_thread = None
        
        # 安全参数
        self.max_velocity = 0.5  # m/s
        self.max_joint_change_per_cycle = 0.1  # rad
        
        # 位置缓冲（用于速度检查）
        self.master_position_buffer = []
        self.position_buffer_size = 5
        
    def _setup_kinematics(self):
        """设置运动学求解器"""
        # 主机械臂运动学
        master_dh = self.master_robot.get_dh_params()
        self.master_fk = ForwardKinematics(master_dh)
        self.master_ik = InverseKinematics(
            self.master_fk, 
            self.master_robot.joint_limits
        )
        
        # 从机械臂运动学
        slave_dh = self.slave_robot.get_dh_params()
        self.slave_fk = ForwardKinematics(slave_dh)
        self.slave_ik = InverseKinematics(
            self.slave_fk, 
            self.slave_robot.joint_limits
        )
        
    def connect(self) -> bool:
        """连接机械臂"""
        try:
            # 使能机械臂
            if not self.master_robot.enable():
                self.logger.error("主机械臂使能失败")
                return False
                
            if not self.slave_robot.enable():
                self.logger.error("从机械臂使能失败")
                return False
            
            # 回零位
            if not self.master_robot.home():
                self.logger.error("主机械臂回零失败")
                return False
                
            if not self.slave_robot.home():
                self.logger.error("从机械臂回零失败")
                return False
            
            self.is_connected = True
            self.logger.info("机械臂连接成功")
            return True
            
        except Exception as e:
            self.logger.error(f"连接机械臂失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        try:
            if self.is_running:
                self.stop()
            
            self.master_robot.disable()
            self.slave_robot.disable()
            
            # 关闭gym环境
            if hasattr(self.master_robot, 'close'):
                self.master_robot.close()
            if hasattr(self.slave_robot, 'close'):
                self.slave_robot.close()
                
            self.is_connected = False
            self.logger.info("机械臂断开连接")
            
        except Exception as e:
            self.logger.error(f"断开连接失败: {e}")
    
    def start(self) -> bool:
        """开始主从控制"""
        if not self.is_connected:
            self.logger.error("机械臂未连接")
            return False
        
        if self.is_running:
            self.logger.warning("主从控制已在运行")
            return True
        
        try:
            self.is_running = True
            self.emergency_stop_flag = False
            
            # 启动控制线程
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            self.logger.info("主从控制启动成功")
            return True
            
        except Exception as e:
            self.logger.error(f"启动主从控制失败: {e}")
            self.is_running = False
            return False
    
    def stop(self):
        """停止主从控制"""
        try:
            self.is_running = False
            
            # 停止机械臂运动
            self.master_robot.stop_motion()
            self.slave_robot.stop_motion()
            
            # 等待控制线程结束
            if self.control_thread and self.control_thread.is_alive():
                self.control_thread.join(timeout=2.0)
            
            self.logger.info("主从控制停止")
            
        except Exception as e:
            self.logger.error(f"停止主从控制失败: {e}")
    
    def emergency_stop(self):
        """紧急停止"""
        self.emergency_stop_flag = True
        self.master_robot.stop_motion(emergency=True)
        self.slave_robot.stop_motion(emergency=True)
        self.is_running = False
        self.logger.warning("紧急停止激活")
    
    def set_master_to_slave_transform(self, transform: np.ndarray):
        """
        设置主从变换矩阵
        
        Args:
            transform: 4x4变换矩阵
        """
        if transform.shape != (4, 4):
            raise ValueError("变换矩阵必须是4x4")
        
        self.master_to_slave_transform = transform.copy()
        self.logger.info("主从变换矩阵已更新")
    
    def _control_loop(self):
        """主控制循环"""
        self.logger.info("控制循环开始")
        
        while self.is_running and not self.emergency_stop_flag:
            start_time = time.time()
            
            try:
                # 执行一个控制周期
                success = self._control_cycle()
                
                if not success:
                    self.logger.warning("控制周期执行失败")
                    
            except Exception as e:
                self.logger.error(f"控制循环异常: {e}")
                break
            
            # 控制频率
            elapsed_time = time.time() - start_time
            sleep_time = max(0, self.control_dt - elapsed_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        self.logger.info("控制循环结束")
    
    def _control_cycle(self) -> bool:
        """执行单个控制周期"""
        try:
            # 检查紧急停止
            if self.emergency_stop_flag:
                return False
            
            # 1. 获取主机械臂关节状态
            master_joint_positions = self.master_robot.get_joint_positions()
            
            # 检查关节数据有效性
            if np.any(np.isnan(master_joint_positions)) or len(master_joint_positions) == 0:
                self.logger.warning("主机械臂关节数据无效")
                return False
            
            # 2. 通过正向运动学获取主机械臂末端位姿
            master_position, master_orientation = self.master_fk.forward_kinematics(master_joint_positions)
            
            # 更新位置缓冲（用于速度检查）
            self.master_position_buffer.append(master_position)
            if len(self.master_position_buffer) > self.position_buffer_size:
                self.master_position_buffer.pop(0)
            
            # 安全检查：检查主机械臂运动速度
            if len(self.master_position_buffer) >= 2:
                velocity = np.linalg.norm(
                    self.master_position_buffer[-1] - self.master_position_buffer[-2]
                ) / self.control_dt
                
                if velocity > self.max_velocity:
                    self.logger.warning(f"主机械臂运动速度过快: {velocity:.3f} m/s")
                    return True  # 跳过这个周期但不停止
            
            # 3. 应用主从变换计算从机械臂目标位姿
            slave_target_position, slave_target_orientation = self._calculate_slave_target(
                master_position, master_orientation)
            
            # 4. 求解从机械臂逆运动学
            slave_current_joints = self.slave_robot.get_joint_positions()
            
            # 使用优化方法求解IK
            slave_target_joints, ik_success = self.slave_ik.ik_optimization_method(
                slave_target_position, slave_target_orientation, slave_current_joints)
            
            if ik_success:
                # 安全检查：限制关节变化量
                joint_change = np.abs(slave_target_joints - slave_current_joints)
                if np.any(joint_change > self.max_joint_change_per_cycle):
                    self.logger.warning("从机械臂关节变化过大，限制运动")
                    # 限制关节变化
                    change_direction = slave_target_joints - slave_current_joints
                    change_direction = change_direction / (np.linalg.norm(change_direction) + 1e-8)
                    slave_target_joints = slave_current_joints + change_direction * self.max_joint_change_per_cycle
                
                # 5. 控制从机械臂运动
                success = self.slave_robot.move_to_joint_positions(
                    slave_target_joints, 
                    velocity_scale=0.5,  # 降低速度
                    blocking=False
                )
                
                if not success:
                    self.logger.warning("从机械臂运动控制失败")
                    return False
                
                # 输出调试信息（可选）
                if hasattr(self, '_debug_counter'):
                    self._debug_counter += 1
                else:
                    self._debug_counter = 1
                
                if self._debug_counter % 50 == 0:  # 每50个周期输出一次
                    self.logger.info(f"主机械臂位置: {master_position}")
                    self.logger.info(f"从机械臂目标位置: {slave_target_position}")
                
            else:
                self.logger.warning("从机械臂逆运动学求解失败")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"控制周期执行错误: {e}")
            return False
    
    def _calculate_slave_target(self, master_position: np.ndarray, 
                               master_orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算从机械臂目标位姿
        
        Args:
            master_position: 主机械臂位置
            master_orientation: 主机械臂姿态（四元数）
            
        Returns:
            slave_target_position: 从机械臂目标位置
            slave_target_orientation: 从机械臂目标姿态
        """
        # 构建主机械臂位姿矩阵
        from scipy.spatial.transform import Rotation as R
        
        # 四元数转旋转矩阵
        rotation = R.from_quat([master_orientation[1], master_orientation[2], 
                               master_orientation[3], master_orientation[0]])  # [x,y,z,w]
        rotation_matrix = rotation.as_matrix()
        
        master_pose = np.eye(4)
        master_pose[:3, :3] = rotation_matrix
        master_pose[:3, 3] = master_position
        
        # 应用主从变换
        slave_pose = self.master_to_slave_transform @ master_pose
        
        # 提取从机械臂目标位姿
        slave_target_position = slave_pose[:3, 3]
        
        # 旋转矩阵转四元数
        slave_rotation = R.from_matrix(slave_pose[:3, :3])
        slave_quat_xyzw = slave_rotation.as_quat()  # [x,y,z,w]
        slave_target_orientation = np.array([slave_quat_xyzw[3], slave_quat_xyzw[0], 
                                           slave_quat_xyzw[1], slave_quat_xyzw[2]])  # [w,x,y,z]
        
        return slave_target_position, slave_target_orientation
    
    def get_status(self) -> Dict:
        """获取系统状态"""
        try:
            master_joints = self.master_robot.get_joint_positions()
            slave_joints = self.slave_robot.get_joint_positions()
            
            master_pos, master_orn = self.master_robot.get_end_effector_pose()
            slave_pos, slave_orn = self.slave_robot.get_end_effector_pose()
            
            return {
                "connected": self.is_connected,
                "running": self.is_running,
                "emergency_stop": self.emergency_stop_flag,
                "master": {
                    "joints": master_joints.tolist(),
                    "position": master_pos.tolist(),
                    "orientation": master_orn.tolist(),
                    "ready": self.master_robot.is_ready(),
                    "moving": self.master_robot.is_moving()
                },
                "slave": {
                    "joints": slave_joints.tolist(),
                    "position": slave_pos.tolist(),
                    "orientation": slave_orn.tolist(),
                    "ready": self.slave_robot.is_ready(),
                    "moving": self.slave_robot.is_moving()
                }
            }
        except Exception as e:
            return {"error": str(e)}
    
    def render_master(self, mode="human"):
        """渲染主机械臂"""
        if hasattr(self.master_robot, 'render'):
            return self.master_robot.render(mode)
    
    def render_slave(self, mode="human"):
        """渲染从机械臂"""
        if hasattr(self.slave_robot, 'render'):
            return self.slave_robot.render(mode)
    
    def __del__(self):
        """析构函数"""
        try:
            self.disconnect()
 