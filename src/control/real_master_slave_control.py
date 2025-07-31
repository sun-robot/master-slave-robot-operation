"""
真实硬件主从控制模块
专门用于真实D1机械臂的主从协调控制
"""

import numpy as np
import time
import threading
import logging
from typing import Dict, Optional, Callable
from ..robot_models.d1arm_robot import D1ArmHardware
from ..kinematics.forward_kinematics import ForwardKinematics
from ..kinematics.inverse_kinematics import InverseKinematics
from ..robot_models.d1arm_robot import D1ArmModel
from ..utils.math_utils import normalize_quaternion


class RealMasterSlaveController:
    """真实硬件主从控制器"""
    
    def __init__(self, 
                 master_config: Dict, 
                 slave_config: Dict,
                 safety_config: Optional[Dict] = None):
        """
        初始化真实硬件主从控制器
        
        Args:
            master_config: 主机械臂配置
            slave_config: 从机械臂配置 
            safety_config: 安全配置
        """
        # 配置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # 创建机械臂实例
        self.master_robot = D1ArmHardware(
            robot_id=master_config.get('id', 'master_d1arm'),
            connection_config=master_config.get('connection', {}),
            arm_id=master_config.get('arm_id', 1)
        )
        
        self.slave_robot = D1ArmHardware(
            robot_id=slave_config.get('id', 'slave_d1arm'),
            connection_config=slave_config.get('connection', {}),
            arm_id=slave_config.get('arm_id', 2)
        )
        
        # 控制参数
        self.control_frequency = 50.0  # 真实硬件使用较低频率
        self.control_dt = 1.0 / self.control_frequency
        
        # 状态管理
        self.is_running = False
        self.is_connected = False
        self.emergency_stop_flag = False
        
        # 主从变换
        self.master_to_slave_transform = np.eye(4)
        
        # 安全参数
        self.safety_config = safety_config or {}
        self.max_position_error = self.safety_config.get('max_position_error', 0.1)  # 10cm
        self.max_velocity = self.safety_config.get('max_velocity', 0.2)  # 20cm/s
        self.max_acceleration = self.safety_config.get('max_acceleration', 0.5)  # 50cm/s²
        
        # 运动缓冲和滤波
        self.position_buffer_size = 5
        self.master_position_buffer = []
        self.slave_position_buffer = []
        
        # 控制线程
        self.control_thread = None
        self.stop_event = threading.Event()
        
        # 状态回调
        self.status_callback = None
        self.error_callback = None
        
    def set_master_to_slave_transform(self, transform: np.ndarray):
        """设置主从坐标变换"""
        if transform.shape != (4, 4):
            raise ValueError("变换矩阵必须是4x4")
        self.master_to_slave_transform = transform.copy()
        self.logger.info("主从变换矩阵已更新")
    
    def set_status_callback(self, callback: Callable):
        """设置状态回调函数"""
        self.status_callback = callback
    
    def set_error_callback(self, callback: Callable):
        """设置错误回调函数"""
        self.error_callback = callback
    
    def connect_robots(self) -> bool:
        """连接主从机械臂"""
        self.logger.info("正在连接主从机械臂...")
        
        try:
            # 连接主机械臂
            if not self.master_robot.connect():
                self.logger.error("主机械臂连接失败")
                return False
            
            # 连接从机械臂
            if not self.slave_robot.connect():
                self.logger.error("从机械臂连接失败")
                self.master_robot.disconnect()
                return False
            
            # 使能机械臂
            if not (self.master_robot.enable() and self.slave_robot.enable()):
                self.logger.error("机械臂使能失败")
                self.disconnect_robots()
                return False
            
            self.is_connected = True
            self.logger.info("主从机械臂连接成功")
            return True
            
        except Exception as e:
            self.logger.error(f"连接过程中发生错误: {e}")
            self.disconnect_robots()
            return False
    
    def disconnect_robots(self):
        """断开主从机械臂连接"""
        self.logger.info("正在断开主从机械臂连接...")
        
        try:
            self.stop_control()
            
            if hasattr(self, 'master_robot'):
                self.master_robot.disable()
                self.master_robot.disconnect()
            
            if hasattr(self, 'slave_robot'):
                self.slave_robot.disable()
                self.slave_robot.disconnect()
                
            self.is_connected = False
            self.logger.info("主从机械臂已断开连接")
            
        except Exception as e:
            self.logger.error(f"断开连接时发生错误: {e}")
    
    def start_control(self) -> bool:
        """启动主从控制"""
        if not self.is_connected:
            self.logger.error("机械臂未连接，无法启动控制")
            return False
        
        if self.is_running:
            self.logger.warning("控制已经在运行中")
            return True
        
        try:
            # 检查机械臂状态
            if not (self.master_robot.is_ready() and self.slave_robot.is_ready()):
                self.logger.error("机械臂未就绪")
                return False
            
            # 记录初始位置
            master_pos, master_orient = self.master_robot.get_end_effector_pose()
            slave_pos, slave_orient = self.slave_robot.get_end_effector_pose()
            
            self.master_position_buffer = [master_pos] * self.position_buffer_size
            self.slave_position_buffer = [slave_pos] * self.position_buffer_size
            
            # 启动控制线程
            self.stop_event.clear()
            self.emergency_stop_flag = False
            self.is_running = True
            
            self.control_thread = threading.Thread(target=self._control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            
            self.logger.info("主从控制已启动")
            return True
            
        except Exception as e:
            self.logger.error(f"启动控制失败: {e}")
            self.is_running = False
            return False
    
    def stop_control(self):
        """停止主从控制"""
        if not self.is_running:
            return
        
        self.logger.info("正在停止主从控制...")
        
        self.is_running = False
        self.stop_event.set()
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        # 停止机械臂运动
        try:
            self.master_robot.stop_motion()
            self.slave_robot.stop_motion()
        except Exception as e:
            self.logger.error(f"停止机械臂运动时发生错误: {e}")
        
        self.logger.info("主从控制已停止")
    
    def emergency_stop(self):
        """紧急停止"""
        self.logger.warning("执行紧急停止!")
        
        self.emergency_stop_flag = True
        self.stop_event.set()
        
        try:
            self.master_robot.stop_motion(emergency=True)
            self.slave_robot.stop_motion(emergency=True)
        except Exception as e:
            self.logger.error(f"紧急停止时发生错误: {e}")
        
        if self.error_callback:
            self.error_callback("紧急停止已触发")
    
    def _control_loop(self):
        """控制循环主函数"""
        self.logger.info("控制循环开始")
        
        last_time = time.time()
        
        while self.is_running and not self.stop_event.is_set():
            try:
                current_time = time.time()
                actual_dt = current_time - last_time
                last_time = current_time
                
                # 执行控制周期
                success = self._control_cycle()
                
                if not success:
                    self.logger.warning("控制周期执行失败")
                    continue
                
                # 更新状态回调
                if self.status_callback:
                    try:
                        status = self._get_control_status()
                        self.status_callback(status)
                    except Exception as e:
                        self.logger.warning(f"状态回调执行失败: {e}")
                
                # 控制频率
                sleep_time = max(0, self.control_dt - actual_dt)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"控制循环发生错误: {e}")
                if self.error_callback:
                    self.error_callback(f"控制循环错误: {e}")
                break
        
        self.logger.info("控制循环结束")
    
    def _control_cycle(self) -> bool:
        """执行单个控制周期 - 真实demo逻辑：关节状态->末端位姿->关节命令"""
        try:
            # 检查紧急停止
            if self.emergency_stop_flag:
                return False
            
            # 1. 从主机械臂获取关节状态
            master_joint_positions = self.master_robot.get_joint_positions()
            
            # 检查关节数据有效性
            if np.any(np.isnan(master_joint_positions)) or len(master_joint_positions) != 7:
                self.logger.warning("主机械臂关节数据无效")
                return False
            
            # 2. 通过正向运动学转换为主机械臂末端位姿
            from ..kinematics.forward_kinematics import ForwardKinematics
            master_fk = ForwardKinematics(self.master_robot.get_dh_params())
            master_position, master_orientation = master_fk.forward_kinematics(master_joint_positions)
            
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
            
            # 4. 通过逆运动学计算从机械臂目标关节角度
            from ..kinematics.inverse_kinematics import InverseKinematics
            slave_fk = ForwardKinematics(self.slave_robot.get_dh_params())
            slave_ik = InverseKinematics(slave_fk, self.slave_robot.get_joint_limits())
            
            # 获取从机械臂当前关节位置作为初始猜测
            slave_current_joints = self.slave_robot.get_joint_positions()
            
            # 求解逆运动学
            slave_target_joints, ik_success = slave_ik.ik_optimization_method(
                slave_target_position, slave_target_orientation, slave_current_joints)
            
            if not ik_success:
                self.logger.warning("从机械臂逆运动学求解失败")
                return False
            
            # 5. 安全检查：关节变化幅度
            joint_change = np.linalg.norm(slave_target_joints - slave_current_joints)
            max_joint_change = 0.1  # 每个周期最大关节变化（弧度）
            
            if joint_change > max_joint_change:
                self.logger.warning(f"关节变化过大: {joint_change:.3f} rad")
                # 限制关节变化幅度
                direction = (slave_target_joints - slave_current_joints) / joint_change
                slave_target_joints = slave_current_joints + direction * max_joint_change
            
            # 6. 发送关节命令给从机械臂
            success = self.slave_robot.move_to_joint_positions(
                slave_target_joints,
                velocity_scale=0.5,  # 保守的速度设置
                acceleration_scale=0.5,
                blocking=False  # 非阻塞模式
            )
            
            if not success:
                self.logger.warning("从机械臂关节命令发送失败")
                return False
            
            # 记录控制状态（用于调试）
            self.logger.debug(f"主关节: {master_joint_positions}")
            self.logger.debug(f"主位姿: pos={master_position}, orient={master_orientation}")
            self.logger.debug(f"从目标关节: {slave_target_joints}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"控制周期执行错误: {e}")
            return False
    
    def _calculate_slave_target(self, master_position: np.ndarray, 
                              master_orientation: np.ndarray) -> tuple:
        """计算从机械臂目标位姿"""
        # 构建主机械臂位姿矩阵
        master_pose = np.eye(4)
        master_pose[:3, 3] = master_position
        
        # 四元数转旋转矩阵
        w, x, y, z = normalize_quaternion(master_orientation)
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        master_pose[:3, :3] = R
        
        # 应用主从变换
        slave_pose = self.master_to_slave_transform @ master_pose
        
        # 提取从机械臂目标位姿
        slave_target_position = slave_pose[:3, 3]
        
        # 旋转矩阵转四元数
        R_slave = slave_pose[:3, :3]
        trace = np.trace(R_slave)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            slave_target_orientation = np.array([
                0.25 * s,
                (R_slave[2, 1] - R_slave[1, 2]) / s,
                (R_slave[0, 2] - R_slave[2, 0]) / s,
                (R_slave[1, 0] - R_slave[0, 1]) / s
            ])
        else:
            # 处理特殊情况
            slave_target_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        
        return slave_target_position, normalize_quaternion(slave_target_orientation)
    
    def _get_control_status(self) -> Dict:
        """获取控制状态"""
        try:
            # 获取关节状态
            master_joints = self.master_robot.get_joint_positions()
            slave_joints = self.slave_robot.get_joint_positions()
            
            # 通过正向运动学计算末端位姿
            from ..kinematics.forward_kinematics import ForwardKinematics
            master_fk = ForwardKinematics(self.master_robot.get_dh_params())
            slave_fk = ForwardKinematics(self.slave_robot.get_dh_params())
            
            master_pos, master_orient = master_fk.forward_kinematics(master_joints)
            slave_pos, slave_orient = slave_fk.forward_kinematics(slave_joints)
            
            # 计算期望的从机械臂位姿（通过主从变换）
            expected_slave_pos, expected_slave_orient = self._calculate_slave_target(
                master_pos, master_orient)
            
            # 计算跟随误差（使用期望位姿）
            position_error = np.linalg.norm(slave_pos - expected_slave_pos)
            
            # 计算关节空间误差
            joint_error = np.linalg.norm(master_joints - slave_joints)
            
            return {
                'timestamp': time.time(),
                'is_running': self.is_running,
                'is_connected': self.is_connected,
                'emergency_stop': self.emergency_stop_flag,
                'master_robot': {
                    'position': master_pos.tolist(),
                    'orientation': master_orient.tolist(),
                    'joint_positions': master_joints.tolist(),
                    'is_ready': self.master_robot.is_ready(),
                    'is_moving': self.master_robot.is_moving()
                },
                'slave_robot': {
                    'position': slave_pos.tolist(),
                    'orientation': slave_orient.tolist(),
                    'joint_positions': slave_joints.tolist(),
                    'is_ready': self.slave_robot.is_ready(),
                    'is_moving': self.slave_robot.is_moving()
                },
                'expected_slave_position': expected_slave_pos.tolist(),
                'expected_slave_orientation': expected_slave_orient.tolist(),
                'position_error': position_error,
                'joint_error': joint_error,
                'control_frequency': self.control_frequency
            }
        except Exception as e:
            return {
                'timestamp': time.time(),
                'error': f"获取状态失败: {e}"
            }
    
    def get_status(self) -> Dict:
        """获取当前控制状态"""
        return self._get_control_status()
    
    def home_robots(self) -> bool:
        """将两个机械臂回到零位"""
        if not self.is_connected:
            self.logger.error("机械臂未连接")
            return False
        
        try:
            self.logger.info("正在将机械臂回零位...")
            
            success = True
            if not self.master_robot.home():
                self.logger.error("主机械臂回零失败")
                success = False
            
            if not self.slave_robot.home():
                self.logger.error("从机械臂回零失败")
                success = False
            
            if success:
                self.logger.info("机械臂回零完成")
            
            return success
            
        except Exception as e:
            self.logger.error(f"回零过程发生错误: {e}")
            return False
    
    def __del__(self):
        """析构函数"""
        try:
            self.stop_control()
            self.disconnect_robots()
        except:
            pass 