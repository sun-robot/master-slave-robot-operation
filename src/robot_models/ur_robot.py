"""
基于ROS的UR机械臂控制器实现
支持UR3e、UR5、UR10，包含完整的ROS控制功能
"""

import numpy as np
import time
import threading
from typing import Dict, List, Optional, Tuple, Union
from scipy.spatial.transform import Rotation as R

# ROS相关导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.executors import MultiThreadedExecutor
    
    from controller_manager_msgs.srv import SwitchController, LoadController
    from geometry_msgs.msg import Pose, Quaternion
    from tf2_msgs.msg import TFMessage
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from transforms3d.quaternions import quat2mat
    
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # 创建占位符类以避免导入错误
    class Node:
        pass

# IKFast库导入
try:
    from ur_ikfast import ur_kinematics
    IKFAST_AVAILABLE = True
except ImportError:
    IKFAST_AVAILABLE = False

from .robot_interface import RobotInterface, RobotType, RobotState, EndEffectorType


class URController(Node):
    """UR机械臂ROS控制器节点"""
    
    # 关节名称定义
    JOINT_NAMES = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    
    # 控制器类型配置
    JOINT_TRAJECTORY_CONTROLLERS = [
        "scaled_joint_trajectory_controller",
        "pos_joint_traj_controller"
    ]
    
    CARTESIAN_TRAJECTORY_CONTROLLERS = [
        "forward_cartesian_traj_controller"
    ]
    
    CONFLICTING_CONTROLLERS = ["joint_group_vel_controller"]
    
    def __init__(self, robot_type: RobotType, node_name: str = 'ur_robot_controller'):
        super().__init__(node_name)
        
        self.robot_type = robot_type
        
        # 加载机械臂模型 - 暂时注释掉，因为URModel未定义
        # self._model = URModel(robot_type)
        self._model = None
        
        # 机械臂臂长参数
        if robot_type == RobotType.UR3E:
            self.arm_length = 0.6
            self.min_reach = 0.2
            self.max_height = 0.8
            self.min_height = 0.1
        elif robot_type == RobotType.UR5:
            self.arm_length = 0.85
            self.min_reach = 0.3
            self.max_height = 1.0
            self.min_height = 0.1
        elif robot_type == RobotType.UR10:
            self.arm_length = 1.3
            self.min_reach = 0.5
            self.max_height = 1.5
            self.min_height = 0.1
        else:
            self.arm_length = 0.6
            self.min_reach = 0.2
            self.max_height = 0.8
            self.min_height = 0.1
        
        # 控制器配置
        self.cartesian_controller = self.CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        self.joint_controller = self.JOINT_TRAJECTORY_CONTROLLERS[0]
        self._current_controller = self.joint_controller
        
        # 机械臂状态 - 优化数据存储
        self._ee_pose = Pose()
        self._joint_pose = np.zeros(6)
        self._joint_state_received = False
        self._last_joint_update = 0.0
        self._last_pose_update = 0.0
        self._data_lock = threading.Lock()  # 线程安全锁
        
        # 初始化IKFast求解器
        if IKFAST_AVAILABLE:
            try:
                if robot_type == RobotType.UR3E:
                    self.ur_kinematics = ur_kinematics.URKinematics('ur3e')
                elif robot_type == RobotType.UR5:
                    self.ur_kinematics = ur_kinematics.URKinematics('ur5')
                elif robot_type == RobotType.UR10:
                    self.ur_kinematics = ur_kinematics.URKinematics('ur10')
                else:
                    self.ur_kinematics = None
                self.get_logger().info(f"IKFast求解器初始化成功: {robot_type.value}")
            except Exception as e:
                self.get_logger().warn(f"IKFast求解器初始化失败: {e}")
                self.ur_kinematics = None
        else:
            self.ur_kinematics = None
            self.get_logger().warn("IKFast不可用，将使用数值方法")
        
        # 创建订阅和服务客户端
        self.create_subscription(TFMessage, "/tf", self._update_robot_pose, 1)
        self.create_subscription(JointState, "/joint_states", self._update_joint_pose, 1)
        self.switch_srv = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.load_srv = self.create_client(LoadController, "/controller_manager/load_controller")
        
        # 等待服务就绪
        self._wait_for_service(self.switch_srv, 'switch_controller', timeout_sec=5.0)
        self._wait_for_service(self.load_srv, 'load_controller', timeout_sec=5.0)
        
        # 初始化动作客户端
        self._joint_action_client = ActionClient(
            self, FollowJointTrajectory, 
            f"{self.joint_controller}/follow_joint_trajectory"
        )
        self._cart_action_client = ActionClient(
            self, FollowJointTrajectory,
            f"{self.cartesian_controller}/follow_joint_trajectory"
        )
        
        self.get_logger().info(f"UR {robot_type.value.upper()} ROS控制器初始化完成")
    
    def _wait_for_service(self, client, name, timeout_sec=5.0):
        """带超时的服务等待"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'等待服务 {name}...')
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"服务 {name} 等待超时！")
                return False
        return True
    
    def _update_robot_pose(self, msg):
        """更新末端执行器位姿（TF消息解析）"""
        if not msg.transforms:
            return
        
        try:
            with self._data_lock:
                for tf in msg.transforms:
                    trans = tf.transform.translation
                    # 筛选首个非零位姿
                    if trans.x != 0.0 or trans.y != 0.0 or trans.z != 0.0:
                        self._ee_pose.position.x = trans.x
                        self._ee_pose.position.y = trans.y
                        self._ee_pose.position.z = trans.z
                        self._ee_pose.orientation = tf.transform.rotation
                        self._last_pose_update = time.time()
                        break
        except Exception as e:
            self.get_logger().error(f"TF处理错误: {str(e)}")
    
    def _update_joint_pose(self, msg):
        """更新关节状态（带数据校验）"""
        try:
            if len(msg.position) >= 6:
                with self._data_lock:
                    # 按UR关节顺序重排
                    index_map = [msg.name.index(joint_name) for joint_name in self.JOINT_NAMES]
                    self._joint_pose = np.array([msg.position[i] for i in index_map])
                    self._joint_state_received = True
                    self._last_joint_update = time.time()
        except Exception as e:
            self.get_logger().error(f"关节状态错误: {str(e)}")
    
    def get_joint_pose_rad(self):
        """获取当前关节角度（弧度）"""
        with self._data_lock:
            return self._joint_pose.copy()
    
    def get_joint_pose_deg(self):
        """获取当前关节角度（度数）"""
        with self._data_lock:
            return np.rad2deg(self._joint_pose.copy())
    
    def get_current_pose(self):
        """获取当前末端位姿[7维: x,y,z,qx,qy,qz,qw]"""
        with self._data_lock:
            p = self._ee_pose.position
            o = self._ee_pose.orientation
            return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
    
    def is_data_fresh(self, timeout_sec=1.0):
        """检查数据是否新鲜（在指定时间内有更新）"""
        current_time = time.time()
        joint_fresh = (current_time - self._last_joint_update) < timeout_sec
        pose_fresh = (current_time - self._last_pose_update) < timeout_sec
        return joint_fresh and pose_fresh
    
    def get_current_pose_matrix(self):
        """获取当前末端位姿的4×4齐次变换矩阵"""
        if self.ur_kinematics:
            joint_angles = self.get_joint_pose_rad()
            try:
                matrix = np.eye(4)
                matrix[:3, :4] = self.ur_kinematics.forward(joint_angles, 'matrix')
                return matrix
            except Exception as e:
                self.get_logger().warn(f"FK计算失败: {e}")
                # 回退到TF数据
                return self._get_pose_matrix_from_tf()
        else:
            # 使用TF数据构造矩阵
            return self._get_pose_matrix_from_tf()
    
    def _get_pose_matrix_from_tf(self):
        """从TF数据构造位姿矩阵"""
        with self._data_lock:
            p = self._ee_pose.position
            o = self._ee_pose.orientation
            t = np.array([p.x, p.y, p.z])
            R_mat = quat2mat([o.w, o.x, o.y, o.z])
            matrix = np.eye(4)
            matrix[:3, :3] = R_mat
            matrix[:3, 3] = t
            return matrix
    
    def pose_matrix_to_joint_rad(self, matrix, old_joints=None):
        """将4×4齐次变换矩阵转为关节弧度"""
        if self.ur_kinematics:
            try:
                # 直接使用IKFast
                original_matrix = matrix[:3, :4].copy()
                
                # 检查目标位置是否在合理范围内
                target_position = original_matrix[:3, 3]
                if np.any(np.abs(target_position) > 2.0):  # UR机械臂工作空间大约±2m
                    self.get_logger().warn(f"目标位置 {target_position} 可能超出工作空间")
                
                # 尝试使用初始猜测
                if old_joints is not None:
                    joints = self.ur_kinematics.inverse(original_matrix, q_guess=old_joints)
                    if joints is not None:
                        self.get_logger().info("使用初始猜测成功求解IK")
                        return joints
                
                # 尝试直接求解
                joints = self.ur_kinematics.inverse(original_matrix)
                if joints is not None:
                    self.get_logger().info("直接求解IK成功")
                    return joints
                
                # 如果都失败了，尝试多个初始猜测
                self.get_logger().warn("直接求解失败，尝试多个初始猜测")
                initial_guesses = [
                    np.zeros(6),
                    np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0]),  # 常见的UR姿态
                    np.array([0, 0, 0, 0, 0, 0]),  # 零位
                    np.array([np.pi/2, -np.pi/2, 0, -np.pi/2, 0, 0]),  # 另一个常见姿态
                    np.array([0, -np.pi/4, 0, -np.pi/2, 0, 0]),  # 更保守的姿态
                    np.array([0, -np.pi/3, np.pi/6, -np.pi/2, 0, 0]),  # 另一个保守姿态
                ]
                
                for i, guess in enumerate(initial_guesses):
                    try:
                        joints = self.ur_kinematics.inverse(original_matrix, q_guess=guess)
                        if joints is not None:
                            self.get_logger().info(f"使用初始猜测{i+1}成功求解IK")
                            return joints
                    except Exception as e:
                        self.get_logger().debug(f"初始猜测{i+1}失败: {e}")
                        continue
                
                # 如果还是失败，尝试基于臂长的缩放调整
                self.get_logger().warn("尝试基于臂长的缩放调整...")
                adjusted_matrix = original_matrix.copy()
                
                # 获取目标位置
                target_position = original_matrix[:3, 3]
                target_distance = np.linalg.norm(target_position)
                
                # 使用机械臂的臂长属性
                max_reach = self.arm_length
                min_reach = self.min_reach
                
                # 计算缩放因子
                if target_distance > max_reach:
                    # 如果目标距离超出最大臂长，缩放到最大臂长
                    scale_factor = max_reach / target_distance
                    self.get_logger().info(f"目标距离 {target_distance:.3f}m 超出最大臂长 {max_reach}m，缩放因子: {scale_factor:.3f}")
                elif target_distance < min_reach:
                    # 如果目标距离小于最小臂长，缩放到最小臂长
                    scale_factor = min_reach / target_distance
                    self.get_logger().info(f"目标距离 {target_distance:.3f}m 小于最小臂长 {min_reach}m，缩放因子: {scale_factor:.3f}")
                else:
                    # 在臂长范围内，尝试小幅缩放
                    scale_factors = [0.95, 0.9, 1.05, 1.1]  # 5%和10%的缩放
                    
                    for i, scale in enumerate(scale_factors):
                        scaled_position = target_position * scale
                        adjusted_matrix[:3, 3] = scaled_position
                        
                        try:
                            joints = self.ur_kinematics.inverse(adjusted_matrix, q_guess=np.zeros(6))
                            if joints is not None:
                                self.get_logger().info(f"通过臂长缩放{i+1} (因子: {scale:.3f}) 成功求解IK")
                                return joints
                        except Exception as e:
                            continue
                    
                    # 如果小幅缩放失败，尝试更激进的缩放
                    scale_factors = [0.8, 0.7, 1.2, 1.3]
                    for i, scale in enumerate(scale_factors):
                        scaled_position = target_position * scale
                        adjusted_matrix[:3, 3] = scaled_position
                        
                        try:
                            joints = self.ur_kinematics.inverse(adjusted_matrix, q_guess=np.zeros(6))
                            if joints is not None:
                                self.get_logger().info(f"通过激进缩放{i+1} (因子: {scale:.3f}) 成功求解IK")
                                return joints
                        except Exception as e:
                            continue
                    
                    # 如果还是失败，尝试基于臂长的径向调整
                    self.get_logger().warn("尝试径向调整...")
                    radial_adjustments = [
                        [0.1, 0, 0], [-0.1, 0, 0], [0, 0.1, 0], [0, -0.1, 0],
                        [0, 0, 0.1], [0, 0, -0.1], [0.05, 0.05, 0], [-0.05, -0.05, 0]
                    ]
                    
                    for i, adjustment in enumerate(radial_adjustments):
                        adjusted_position = target_position + np.array(adjustment)
                        adjusted_matrix[:3, 3] = adjusted_position
                        
                        try:
                            joints = self.ur_kinematics.inverse(adjusted_matrix, q_guess=np.zeros(6))
                            if joints is not None:
                                self.get_logger().info(f"通过径向调整{i+1}成功求解IK")
                                return joints
                        except Exception as e:
                            continue
                    
                    return None
                
                # 应用缩放
                scaled_position = target_position * scale_factor
                adjusted_matrix[:3, 3] = scaled_position
                
                try:
                    joints = self.ur_kinematics.inverse(adjusted_matrix, q_guess=np.zeros(6))
                    if joints is not None:
                        self.get_logger().info(f"通过臂长缩放成功求解IK (缩放因子: {scale_factor:.3f})")
                        return joints
                except Exception as e:
                    self.get_logger().debug(f"臂长缩放失败: {e}")
                    pass
                
                self.get_logger().error("所有IK求解尝试都失败了")
                return None
                
            except Exception as e:
                self.get_logger().warn(f"IK求解失败: {e}")
                return None
        else:
            # 回退到数值求解
            self.get_logger().warn("IKFast不可用，使用数值方法")
            return old_joints if old_joints is not None else np.zeros(6)
    
    def joint_to_pose_matrix(self, joint_pose_rad):
        """根据关节角度计算末端执行器的位姿矩阵"""
        if self.ur_kinematics:
            try:
                rot_matrix = self.ur_kinematics.forward(joint_pose_rad, 'matrix')
                transform_matrix = np.eye(4)
                transform_matrix[:3, :4] = rot_matrix
                
                # 根据实际坐标系调整
                # 如果FK计算的位置与实际位置符号相反，则翻转坐标
                actual_pos = self.get_current_pose()[:3]
                fk_pos = transform_matrix[:3, 3]
                
                # 检查是否需要翻转坐标
                if np.any(np.sign(actual_pos) != np.sign(fk_pos)):
                    # 翻转X和Y坐标以匹配实际坐标系
                    transform_matrix[0, 3] = -transform_matrix[0, 3]
                    transform_matrix[1, 3] = -transform_matrix[1, 3]
                
                return transform_matrix
            except Exception as e:
                self.get_logger().warn(f"FK计算失败: {e}")
                return np.eye(4)
        else:
            # 使用DH参数计算
            self.get_logger().warn("IKFast不可用，使用DH参数计算")
            return np.eye(4)
    
    def execute_joint_trajectory_rad(self, joint_pose_rad, max_wait_sec=10.0):
        """执行关节空间轨迹（弧度输入）"""
        success = self._execute_joint_trajectory_internal(joint_pose_rad, max_wait_sec)
        if success:
            return self.wait_for_joint_convergence(joint_pose_rad)
        return False
    
    def execute_joint_trajectory_deg(self, joint_pose_deg, max_wait_sec=10.0):
        """执行关节空间轨迹（度数输入）"""
        joint_pose_rad = np.deg2rad(joint_pose_deg)
        return self.execute_joint_trajectory_rad(joint_pose_rad, max_wait_sec)
    
    def _execute_joint_trajectory_internal(self, joint_pose_rad, max_wait_sec=10.0):
        """关节轨迹执行内部实现"""
        # 输入校验
        if joint_pose_rad is None:
            self.get_logger().error("关节位姿不能为None")
            return False
        
        if joint_pose_rad.shape != (6,):
            self.get_logger().error("关节位姿必须是6维向量")
            return False
        
        # 确保关节控制器激活
        if not self._ensure_controller_active(self.joint_controller):
            self.get_logger().error("关节控制器激活失败")
            return False
        
        # 等待动作服务器就绪
        if not self._joint_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("动作服务器未响应")
            return False
        
        # 构建轨迹目标
        current_pose = self.get_joint_pose_rad()
        time_from_start = max(1.0, np.max(np.abs(joint_pose_rad - current_pose)) * 2.0)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_pose_rad.tolist()
        point.time_from_start = Duration(seconds=time_from_start).to_msg()
        goal.trajectory.points.append(point)
        
        # 异步发送并等待结果
        future = self._joint_action_client.send_goal_async(goal)
        return self._wait_for_future(future, max_wait_sec)
    
    def _wait_for_future(self, future, timeout):
        """非阻塞等待Future结果"""
        start = time.time()
        while time.time() - start < timeout:
            if future.done():
                return future.result().status
            time.sleep(0.05)
        return False
    
    def wait_for_joint_convergence(self, target_rad, tolerance=0.01, timeout=30.0):
        """等待关节收敛到目标位置"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_rad = self.get_joint_pose_rad()
            if np.allclose(current_rad, target_rad, atol=tolerance):
                self.get_logger().info("关节已到达目标位置")
                return True
            time.sleep(0.1)
        self.get_logger().warn("关节未在指定时间内到达目标位置")
        return False
    
    def get_robot_status(self):
        """获取机器人状态信息"""
        status = {
            'robot_type': self.robot_type.value,
            'joint_angles_deg': self.get_joint_pose_deg().tolist(),
            'joint_angles_rad': self.get_joint_pose_rad().tolist(),
            'end_effector_pose': self.get_current_pose().tolist(),
            'ikfast_available': self.ur_kinematics is not None,
            'data_fresh': self.is_data_fresh(),
            'joint_state_received': self._joint_state_received,
            'current_controller': self._current_controller
        }
        return status
    
    def execute_pose_matrix(self, target_matrix, old_joints=None, max_wait_sec=15.0):
        """执行4×4矩阵位姿"""
        joint_target_rad = self.pose_matrix_to_joint_rad(target_matrix, old_joints)
        
        # 检查逆运动学求解是否成功
        if joint_target_rad is None:
            self.get_logger().error("逆运动学求解失败，无法执行位姿控制")
            return False
        
        return self.execute_joint_trajectory_rad(joint_target_rad, max_wait_sec)
    
    def move_to_home(self):
        """移动到Home位置"""
        # 定义默认的home位置（所有关节为0）
        home_pose = np.zeros(6)
        return self.execute_joint_trajectory_rad(home_pose)
    
    def _ensure_controller_active(self, target_controller):
        """确保目标控制器已激活"""
        if self._current_controller == target_controller:
            return True
        
        self.get_logger().info(f"切换控制器: {target_controller}")
        
        # 加载控制器
        load_req = LoadController.Request()
        load_req.name = target_controller
        load_future = self.load_srv.call_async(load_req)
        rclpy.spin_until_future_complete(self, load_future)
        if not load_future.result().ok:
            self.get_logger().error(f"控制器加载失败: {target_controller}")
            return False
        
        # 停止冲突控制器
        stop_controllers = list(
            set(self.JOINT_TRAJECTORY_CONTROLLERS + 
                self.CARTESIAN_TRAJECTORY_CONTROLLERS + 
                self.CONFLICTING_CONTROLLERS) - {target_controller}
        )
        
        # 发送切换请求
        switch_req = SwitchController.Request()
        switch_req.stop_controllers = stop_controllers
        switch_req.start_controllers = [target_controller]
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        switch_future = self.switch_srv.call_async(switch_req)
        rclpy.spin_until_future_complete(self, switch_future)
        
        if not switch_future.result().ok:
            self.get_logger().error("控制器切换失败")
            return False
        
        self._current_controller = target_controller
        time.sleep(0.5)  # 等待控制器初始化
        return True


class URRobot(RobotInterface):
    """
    基于ROS的UR机械臂实现
    支持UR3e、UR5、UR10的完整ROS控制功能
    
    注意：此类专门用于真机控制，所有控制都通过ROS进行。
    不需要IP地址，完全依赖ROS话题和服务进行通信。
    仿真请使用SimulatedRobotInterface。
    """
    
    def __init__(self, robot_type: RobotType, robot_id: str = "ur_robot",
                 node_name: str = 'ur_robot_controller',
                 auto_init_ros: bool = True):
        """
        初始化UR ROS机械臂
        
        Args:
            robot_type: UR机械臂类型 (UR3E, UR5, UR10)
            robot_id: 机械臂唯一标识符
            node_name: ROS节点名称
            auto_init_ros: 是否自动初始化ROS
            
        注意：此实现完全基于ROS控制，不需要IP地址。
        确保ROS环境和UR ROS驱动已正确配置。
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS 2相关包未安装，无法使用ROS功能")
        
        if robot_type not in [RobotType.UR3E, RobotType.UR5, RobotType.UR10]:
            raise ValueError(f"不支持的UR机械臂类型: {robot_type}")
        
        super().__init__(robot_type, robot_id)
        
        # 初始化ROS
        if auto_init_ros and not rclpy.ok():
            rclpy.init()
        
        # 创建ROS控制器节点
        self._ros_controller = URController(robot_type, node_name)
        # self._model = URModel(robot_type)  # 暂时注释掉
        self._model = None
        
        # 启动ROS spin线程
        self._spin_thread = None
        self._executor = None
        if auto_init_ros:
            self._start_ros_thread()
        
        # 等待初始数据
        self._wait_for_initial_data()
    
    def _start_ros_thread(self):
        """启动ROS spin线程"""
        try:
            print("正在启动ROS线程...")
            
            # 创建多线程执行器
            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self._ros_controller)
            
            # 创建并启动线程
            self._spin_thread = threading.Thread(target=self._executor.spin, name="ROS_Spin_Thread")
            self._spin_thread.daemon = True
            self._spin_thread.start()
            
            # 等待线程启动
            time.sleep(0.1)
            if self._spin_thread.is_alive():
                self._ros_controller.get_logger().info("ROS线程启动成功")
                print("ROS线程启动成功")
            else:
                raise RuntimeError("ROS线程启动失败")
                
        except Exception as e:
            error_msg = f"ROS线程启动失败: {e}"
            self._ros_controller.get_logger().error(error_msg)
            print(error_msg)
            
            # 清理资源
            if self._executor:
                try:
                    self._executor.shutdown(wait=False)
                except:
                    pass
                self._executor = None
            
            if self._spin_thread:
                self._spin_thread = None
            
            raise RuntimeError(error_msg)
    
    def _wait_for_initial_data(self, timeout=5.0):
        """等待初始关节状态数据"""
        start_time = time.time()
        while not self._ros_controller._joint_state_received and time.time() - start_time < timeout:
            self._ros_controller.get_logger().info("等待关节状态...")
            time.sleep(0.5)
        
        if not self._ros_controller._joint_state_received:
            self._ros_controller.get_logger().error("启动超时：未收到关节状态")
            self._set_state(RobotState.ERROR, "未收到关节状态数据")
        else:
            self._ros_controller.get_logger().info("关节状态数据接收成功")
            self._set_state(RobotState.IDLE, "机器人就绪")
    
    # ===================== 基本属性实现 =====================
    
    @property
    def name(self) -> str:
        """机械臂名称"""
        return f"Universal Robots {self.robot_type.value.upper()} (ROS)"
    
    @property
    def dof(self) -> int:
        """自由度数量"""
        return 6
    
    @property
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        return URController.JOINT_NAMES
    
    @property
    def joint_limits(self) -> np.ndarray:
        """关节限位"""
        # 暂时返回默认值，因为URModel未定义
        return np.array([[-2*np.pi, 2*np.pi]] * 6)
    
    @property
    def velocity_limits(self) -> np.ndarray:
        """关节速度限位"""
        # 暂时返回默认值，因为URModel未定义
        return np.array([2.0] * 6)
    
    @property
    def acceleration_limits(self) -> np.ndarray:
        """关节加速度限位"""
        # 暂时返回默认值，因为URModel未定义
        return np.array([1.0] * 6)
    
    @property
    def arm_length(self) -> float:
        """机械臂臂长（基座到末端执行器的最大距离）"""
        if self.robot_type == RobotType.UR3E:
            return 0.6
        elif self.robot_type == RobotType.UR5:
            return 0.85
        elif self.robot_type == RobotType.UR10:
            return 1.3
        else:
            return 0.6  # 默认值
    
    @property
    def min_reach(self) -> float:
        """机械臂最小工作距离"""
        if self.robot_type == RobotType.UR3E:
            return 0.2
        elif self.robot_type == RobotType.UR5:
            return 0.3
        elif self.robot_type == RobotType.UR10:
            return 0.5
        else:
            return 0.2  # 默认值
    
    @property
    def max_height(self) -> float:
        """机械臂最大工作高度"""
        if self.robot_type == RobotType.UR3E:
            return 0.8
        elif self.robot_type == RobotType.UR5:
            return 1.0
        elif self.robot_type == RobotType.UR10:
            return 1.5
        else:
            return 0.8  # 默认值
    
    @property
    def min_height(self) -> float:
        """机械臂最小工作高度"""
        return 0.1  # 所有UR机械臂的最小高度基本相同
    
    # ===================== DH参数和运动学实现 =====================
    
    def get_dh_params(self) -> np.ndarray:
        """获取DH参数"""
        # 暂时返回默认值，因为URModel未定义
        return np.zeros((6, 4))
    
    def get_base_transform(self) -> np.ndarray:
        """获取基座变换矩阵"""
        return np.eye(4)
    
    def get_end_effector_transform(self) -> np.ndarray:
        """获取末端执行器变换"""
        return np.eye(4)
    
    # ===================== 状态获取实现 =====================
    
    def get_joint_positions(self) -> np.ndarray:
        """获取当前关节位置"""
        return self._ros_controller.get_joint_pose_rad()
    
    def get_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        # ROS控制器暂不支持速度反馈
        return np.zeros(self.dof)
    
    def get_joint_torques(self) -> np.ndarray:
        """获取当前关节力矩"""
        # ROS控制器暂不支持力矩反馈
        return np.zeros(self.dof)
    
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取末端执行器位姿"""
        pose_7d = self._ros_controller.get_current_pose()
        position = pose_7d[:3]
        orientation = np.array([pose_7d[6], pose_7d[3], pose_7d[4], pose_7d[5]])  # [w,x,y,z]
        return position, orientation
    
    # ===================== 运动控制实现 =====================
    
    def move_to_joint_positions(self, positions: np.ndarray, 
                               velocity_scale: float = 1.0,
                               acceleration_scale: float = 1.0,
                               blocking: bool = True) -> bool:
        """移动到指定关节位置"""
        if not self.is_ready():
            return False
        
        if not self._validate_joint_positions(positions):
            return False
        
        success = self._ros_controller.execute_joint_trajectory_rad(positions)
        if success:
            self._set_state(RobotState.IDLE)
        else:
            self._set_state(RobotState.ERROR, "关节运动失败")
        
        return success
    
    def move_to_pose(self, position: np.ndarray, orientation: np.ndarray,
                    velocity_scale: float = 1.0,
                    acceleration_scale: float = 1.0,
                    blocking: bool = True) -> bool:
        """移动到指定末端位姿"""
        if not self.is_ready():
            return False
        
        # 检查工作空间边界
        original_position = position.copy()
        if not self.check_workspace_bounds(position):
            print(f"⚠️ 目标位置 {position} 超出工作空间，尝试应用臂长缩放...")
            
            # 应用基于臂长的缩放
            distance = np.linalg.norm(position)
            if distance > self.arm_length:
                scale_factor = self.arm_length / distance
                position = position * scale_factor
                print(f"  缩放因子: {scale_factor:.3f}")
                print(f"  缩放后位置: {position}")
                print(f"  缩放后距离: {np.linalg.norm(position):.3f}m")
            elif distance < self.min_reach:
                scale_factor = self.min_reach / distance
                position = position * scale_factor
                print(f"  缩放因子: {scale_factor:.3f}")
                print(f"  缩放后位置: {position}")
                print(f"  缩放后距离: {np.linalg.norm(position):.3f}m")
            
            # 再次检查缩放后的位置
            if not self.check_workspace_bounds(position):
                print(f"❌ 缩放后位置 {position} 仍超出工作空间")
                return False
            else:
                print(f"✅ 缩放后位置在工作空间内")
        
        # 构造变换矩阵
        transform = np.eye(4)
        transform[:3, 3] = position
        
        # 四元数转旋转矩阵
        rot = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])  # [x,y,z,w]
        transform[:3, :3] = rot.as_matrix()
        
        # 获取当前关节角度作为初始猜测
        current_joints = self.get_joint_positions()
        
        success = self._ros_controller.execute_pose_matrix(transform, current_joints)
        if success:
            self._set_state(RobotState.IDLE)
        else:
            self._set_state(RobotState.ERROR, "位姿运动失败")
        
        return success
    
    def move_linear(self, target_position: np.ndarray, 
                   target_orientation: np.ndarray,
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   blocking: bool = True) -> bool:
        """直线运动到目标位姿"""
        # ROS控制器中暂未实现直线插值
        return self.move_to_pose(target_position, target_orientation)
    
    def stop_motion(self, emergency: bool = False) -> bool:
        """停止运动"""
        # ROS控制器中暂未实现运动停止
        if emergency:
            self._set_state(RobotState.EMERGENCY_STOP)
        return True
    
    # ===================== 配置和校准 =====================
    
    def home(self) -> bool:
        """回零位"""
        if not self.is_ready():
            return False
        
        success = self._ros_controller.move_to_home()
        if success:
            self._set_state(RobotState.IDLE)
        
        return success
    
    def calibrate(self) -> bool:
        """校准机械臂"""
        return True  # UR机械臂一般不需要额外校准
    
    # ===================== 安全和状态管理 =====================
    
    def enable(self) -> bool:
        """使能机械臂"""
        self._set_state(RobotState.IDLE)
        return True
    
    def disable(self) -> bool:
        """失能机械臂"""
        self._set_state(RobotState.DISABLED)
        return True
    
    def is_ready(self) -> bool:
        """检查机械臂是否就绪"""
        return (self._ros_controller._joint_state_received and 
                self.state in [RobotState.IDLE, RobotState.MOVING])
    
    def is_moving(self) -> bool:
        """检查机械臂是否在运动"""
        return self.state == RobotState.MOVING
    
    # ===================== 高级功能 =====================
    
    def solve_ik(self, target_pose: np.ndarray, current_joint_angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """求解逆运动学"""
        # 使用提供的4x4位姿矩阵
        transform = target_pose.copy()
        
        # 如果没有提供当前关节角度，使用当前位置
        if current_joint_angles is None:
            current_joint_angles = self.get_joint_positions()
        
        # 尝试多个初始猜测以提高成功率
        guesses = [current_joint_angles]
        guesses.extend([
            np.zeros(6),  # 零位
            np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0]),  # 常见姿态
            np.array([0, -np.pi/2, np.pi/2, 0, 0, 0]),   # 另一个常见姿态
        ])
        
        # 尝试所有猜测
        for guess in guesses:
            joint_angles = self._ros_controller.pose_matrix_to_joint_rad(transform, guess)
            if joint_angles is not None:
                # 验证解的有效性
                return joint_angles, True
                    
        
        return None, False
    
    def _validate_ik_solution(self, joint_angles: np.ndarray, target_position: np.ndarray, 
                             target_orientation: np.ndarray, position_tolerance: float = 0.05,
                             orientation_tolerance: float = 0.5) -> bool:
        """验证IK解的有效性"""
        try:
            # 使用FK计算实际位姿
            actual_matrix = self._ros_controller.joint_to_pose_matrix(joint_angles)
            actual_position = actual_matrix[:3, 3]
            actual_orientation = R.from_matrix(actual_matrix[:3, :3]).as_quat()
            
            # 检查位置误差
            position_error = np.linalg.norm(actual_position - target_position)
            if position_error > position_tolerance:
                print(f"位置误差过大: {position_error:.6f} > {position_tolerance}")
                return False
            
            # 检查方向误差
            if len(target_orientation) == 4:
                target_quat = np.array([target_orientation[1], target_orientation[2], 
                                       target_orientation[3], target_orientation[0]])
            else:
                target_quat = target_orientation
            
            orientation_error = np.linalg.norm(actual_orientation - target_quat)
            if orientation_error > orientation_tolerance:
                print(f"方向误差过大: {orientation_error:.6f} > {orientation_tolerance}")
                return False
            
            return True
        except Exception as e:
            print(f"验证IK解时出错: {e}")
            return False
    
    # ===================== 末端执行器控制 =====================
    
    def get_end_effector_type(self) -> EndEffectorType:
        """获取末端执行器类型"""
        return EndEffectorType.CUSTOM  # UR通常配置定制末端执行器
    
    # ===================== ROS特有功能 =====================
    
    def get_current_pose_matrix(self) -> np.ndarray:
        """获取当前4x4位姿矩阵"""
        return self._ros_controller.get_current_pose_matrix()
    
    def execute_pose_matrix(self, matrix: np.ndarray, old_joints: Optional[np.ndarray] = None) -> bool:
        """执行4×4矩阵位姿"""
        if not self.is_ready():
            return False
        
        if old_joints is None:
            old_joints = self.get_joint_positions()
        
        return self._ros_controller.execute_pose_matrix(matrix, old_joints)
    
    def get_ros_controller(self) -> URController:
        """获取底层ROS控制器（用于高级操作）"""
        return self._ros_controller
    
    def shutdown(self):
        """关闭ROS资源"""
        try:
            print("正在关闭UR机器人ROS资源...")
            
            # 首先停止机械臂运动
            try:
                self.stop_motion(emergency=True)
            except Exception as e:
                print(f"停止机械臂运动时出错: {e}")
            
            # 关闭ROS执行器和线程
            if self._executor:
                try:
                    # 停止执行器 - 修复参数问题
                    self._executor.shutdown()
                    print("ROS执行器已关闭")
                except Exception as e:
                    print(f"关闭ROS执行器时出错: {e}")
                
                # 移除节点
                try:
                    self._executor.remove_node(self._ros_controller)
                except Exception as e:
                    print(f"移除ROS节点时出错: {e}")
            
            # 销毁ROS控制器节点
            if self._ros_controller:
                try:
                    self._ros_controller.destroy_node()
                    print("ROS控制器节点已销毁")
                except Exception as e:
                    print(f"销毁ROS节点时出错: {e}")
            
            # 等待线程结束 - 改进线程终止机制
            if self._spin_thread and self._spin_thread.is_alive():
                try:
                    # 给线程更多时间正常结束
                    self._spin_thread.join(timeout=5.0)
                    if self._spin_thread.is_alive():
                        print("警告: ROS线程未能正常结束，但程序将继续退出")
                    else:
                        print("ROS线程已正常结束")
                except Exception as e:
                    print(f"等待线程结束时出错: {e}")
            
            # 清理引用
            self._executor = None
            self._spin_thread = None
            self._ros_controller = None
            
            self._set_state(RobotState.DISABLED, "机器人已关闭")
            print("UR机器人ROS资源清理完成")
            
        except Exception as e:
            print(f"关闭机器人时出错: {e}")
            import traceback
            traceback.print_exc()
    
    # ===================== 新增实用方法 =====================
    
    def get_robot_status(self):
        """获取机器人完整状态信息"""
        status = self._ros_controller.get_robot_status()
        status.update({
            'robot_id': self.robot_id,
            'state': self._state.value,
            'error_message': self._error_message,
            'is_ready': self.is_ready(),
            'is_moving': self.is_moving(),
            'arm_length': self.arm_length,
            'min_reach': self.min_reach,
            'max_height': self.max_height,
            'min_height': self.min_height
        })
        return status
    
    def wait_for_data_fresh(self, timeout_sec=10.0):
        """等待数据新鲜（在指定时间内有更新）"""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self._ros_controller.is_data_fresh():
                return True
            time.sleep(0.1)
        return False
    
    def get_current_joint_angles_deg(self):
        """获取当前关节角度（度数）"""
        return self._ros_controller.get_joint_pose_deg()
    
    def get_current_joint_angles_rad(self):
        """获取当前关节角度（弧度）"""
        return self._ros_controller.get_joint_pose_rad()
    def get_current_end_effector_pose(self):
        """获取当前末端位置（毫米）"""
        pose_7d = self._ros_controller.get_current_pose()
        return pose_7d[:3]
    def get_current_end_effector_pose_mm(self):
        """获取当前末端位置（毫米）"""
        position_mm = self.get_current_end_effector_pose * 1000  # 转换为毫米
        return position_mm
    
    def check_workspace_bounds(self, target_position: np.ndarray) -> bool:
        """
        检查目标位置是否在UR机械臂工作空间内（基于臂长）
        
        Args:
            target_position: 目标位置 [x, y, z]
            
        Returns:
            是否在工作空间内
        """
        # 计算目标距离
        target_distance = np.linalg.norm(target_position)
        
        # 使用机械臂的臂长属性
        max_reach = self.arm_length
        min_reach = self.min_reach
        max_height = self.max_height
        min_height = self.min_height
        
        # 检查径向距离
        if target_distance > max_reach:
            print(f"⚠️ 目标位置 {target_position} 距离 {target_distance:.3f}m 超出最大臂长 {max_reach}m")
            return False
        elif target_distance < min_reach:
            print(f"⚠️ 目标位置 {target_position} 距离 {target_distance:.3f}m 小于最小臂长 {min_reach}m")
            return False
        
        # 检查高度范围
        if target_position[2] > max_height:
            print(f"⚠️ 目标位置 {target_position} 高度 {target_position[2]:.3f}m 超出最大高度 {max_height}m")
            return False
        elif target_position[2] < min_height:
            print(f"⚠️ 目标位置 {target_position} 高度 {target_position[2]:.3f}m 低于最小高度 {min_height}m")
            return False
        
        return True
    
    def print_status(self):
        """打印机器人状态信息"""
        status = self.get_robot_status()
        print(f"\n=== UR机器人状态 ===")
        print(f"机器人类型: {status['robot_type']}")
        print(f"机器人ID: {status['robot_id']}")
        print(f"状态: {status['state']} - {status['error_message']}")
        print(f"就绪: {status['is_ready']}")
        print(f"运动: {status['is_moving']}")
        print(f"数据新鲜: {status['data_fresh']}")
        print(f"关节角度 (度): {status['joint_angles_deg']}")
        print(f"末端位置 (毫米): {self.get_current_end_effector_pose_mm()}")
        print(f"IKFast可用: {status['ikfast_available']}")
        print(f"当前控制器: {status['current_controller']}")
        print(f"臂长: {self.arm_length}m")
        print(f"工作范围: {self.min_reach}m - {self.arm_length}m")
        print(f"高度范围: {self.min_height}m - {self.max_height}m")
        print("=" * 30)
    
    def test_workspace_points(self):
        """测试工作空间内的多个点"""
        print("\n=== 工作空间测试 ===")
        
        # 定义一些可能在工作空间内的点
        test_points = [
            # 当前位置附近
            (self.get_current_end_effector_pose_mm() / 1000.0, [1.0, 0.0, 0.0, 0.0]),
            # 常见工作空间点
            (np.array([0.3, 0.0, 0.4]), [1.0, 0.0, 0.0, 0.0]),
            (np.array([0.4, 0.0, 0.3]), [1.0, 0.0, 0.0, 0.0]),
            (np.array([0.2, 0.2, 0.3]), [1.0, 0.0, 0.0, 0.0]),
            (np.array([0.2, -0.2, 0.3]), [1.0, 0.0, 0.0, 0.0]),
            # 更保守的点
            (np.array([0.1, 0.0, 0.2]), [1.0, 0.0, 0.0, 0.0]),
            (np.array([0.15, 0.0, 0.25]), [1.0, 0.0, 0.0, 0.0]),
        ]
        
        success_count = 0
        for i, (position, orientation) in enumerate(test_points):
            print(f"\n测试点 {i+1}: 位置={position}, 方向={orientation}")
            
            # 构造4x4位姿矩阵
            target_pose = np.eye(4)
            target_pose[:3, 3] = position
            # 将四元数转换为旋转矩阵
            from scipy.spatial.transform import Rotation as R
            rot = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
            target_pose[:3, :3] = rot.as_matrix()
            
            joint_solution, success = self.solve_ik(target_pose, self.get_joint_positions())
            if success:
                print(f"✅ IK求解成功: {np.rad2deg(joint_solution)}°")
                success_count += 1
            else:
                print("❌ IK求解失败")
        
        print(f"\n工作空间测试结果: {success_count}/{len(test_points)} 成功")
        return success_count > 0 